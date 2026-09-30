"""M02 bounded scan-stamp/TF rendezvous; pure Python, no ROS or learned code.

The original six-scan history and >= 0.20 s source-stamp sampling interval are
retained, including the legacy floating source-stamp subtraction for sampling.
The unchanged 0.20 s age ceiling uses integer nanoseconds to distinguish its
inclusive boundary. A scan is committed only with its own timestamp transform.
Temporary TF unavailability is a wait, not a terminal transform failure.
"""
from collections import Counter, deque
from dataclasses import dataclass
import math


class TransformPending(Exception):
    """TF at the requested scan stamp has not arrived yet."""


def _ns(seconds):
    seconds = float(seconds)
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('invalid nonnegative timestamp')
    return int(round(seconds*1_000_000_000))


@dataclass
class _PendingScan:
    stamp_ns: int
    received_ns: int
    payload: object
    source_stamp_s: object = None
    first_tf_wait_ns: object = None
    tf_attempts: int = 0


class TimestampedScanQueue:
    MAXIMUM_AGE_NS = 200_000_000
    SAMPLE_INTERVAL_NS = 200_000_000
    HISTORY_SIZE = 6
    MAXIMUM_PENDING = 8

    def __init__(self):
        self.pending = deque()
        self.history_stamps_ns = deque(maxlen=self.HISTORY_SIZE)
        self.history_source_stamps_s = deque(maxlen=self.HISTORY_SIZE)
        self.last_received_stamp_ns = None
        self.last_now_ns = None
        self.fault_reason = None
        self.epoch = 0
        self.counts = Counter()
        self.events = []
        self.commit_audit = []
        self.maximum_queue_depth = 0

    def _event(self, reason, now_ns, entry=None):
        event = {'reason': reason, 'now_s': now_ns/1e9, 'epoch': self.epoch}
        if entry is not None:
            event['stamp_s'] = entry.stamp_ns/1e9
            event['queue_residence_s'] = (now_ns-entry.received_ns)/1e9
            event['tf_attempts'] = entry.tf_attempts
            event['tf_wait_s'] = (None if entry.first_tf_wait_ns is None
                                  else (now_ns-entry.first_tf_wait_ns)/1e9)
        self.events.append(event)

    def observe_time(self, now_s):
        now = _ns(now_s)
        if self.last_now_ns is not None and now < self.last_now_ns:
            self.counts['clock_reset_count'] += 1
            self.counts['clock_reset_discarded_pending'] += len(self.pending)
            self.pending.clear()
            self.history_stamps_ns.clear()
            self.history_source_stamps_s.clear()
            self.last_received_stamp_ns = None
            self.epoch += 1
            self.fault_reason = 'clock_reset_requires_new_episode'
            self._event('clock_reset', now)
        self.last_now_ns = now
        return now

    def submit(self, stamp_s, payload, now_s):
        now = self.observe_time(now_s)
        self.counts['received_count'] += 1
        if self.fault_reason:
            self.counts['fault_rejected_count'] += 1
            return False
        try:
            stamp = _ns(stamp_s)
            if stamp == 0:
                raise ValueError('zero stamp would request latest TF')
        except (ValueError, TypeError):
            self.counts['invalid_stamp_count'] += 1
            self._event('invalid_scan_stamp', now)
            return False
        if self.last_received_stamp_ns is not None and stamp <= self.last_received_stamp_ns:
            self.counts['out_of_order_count'] += 1
            self._event('duplicate_or_out_of_order_scan', now, _PendingScan(stamp,now,payload))
            return False
        self.last_received_stamp_ns = stamp
        entry = _PendingScan(stamp, now, payload, float(stamp_s))
        if now-stamp > self.MAXIMUM_AGE_NS:
            self.counts['expired_count'] += 1
            self._event('expired_on_arrival', now, entry)
            return False
        if stamp-now > self.MAXIMUM_AGE_NS:
            self.counts['future_stamp_rejected_count'] += 1
            self._event('scan_stamp_far_ahead_of_clock', now, entry)
            return False
        if len(self.pending) >= self.MAXIMUM_PENDING:
            self.counts['overflow_count'] += 1
            self._event('bounded_queue_overflow', now, entry)
            return False
        self.pending.append(entry)
        self.maximum_queue_depth = max(self.maximum_queue_depth, len(self.pending))
        return True

    def drain(self, now_s, resolve_transform, commit):
        """resolve(payload, exact_stamp_s) -> TF; commit(payload, TF, stamp_s).

        Only TransformPending is retryable. Other resolver/commit exceptions
        latch a visible fault; they are never silently reclassified as TF waits.
        Callers serialize this method with submit and runtime observation use.
        """
        now = self.observe_time(now_s)
        committed = 0
        if self.fault_reason:
            return committed
        while self.pending:
            entry = self.pending[0]
            last = self.history_source_stamps_s[-1] if self.history_source_stamps_s else None
            if last is not None and entry.source_stamp_s-last < .20:
                self.pending.popleft()
                self.counts['subsampled_count'] += 1
                continue
            if now-entry.stamp_ns > self.MAXIMUM_AGE_NS:
                self.pending.popleft()
                self.counts['expired_count'] += 1
                self._event('expired_waiting_for_transform' if entry.tf_attempts else 'expired_before_transform',now,entry)
                continue
            if entry.stamp_ns > now:
                self.counts['clock_catchup_wait_polls'] += 1
                break
            entry.tf_attempts += 1
            try:
                transform = resolve_transform(entry.payload, entry.source_stamp_s)
            except TransformPending:
                self.counts['tf_wait_poll_count'] += 1
                if entry.first_tf_wait_ns is None:
                    entry.first_tf_wait_ns = now
                    self.counts['tf_wait_scan_count'] += 1
                break
            except Exception as exc:
                self.fault_reason = 'transform_resolver_error:'+type(exc).__name__
                self.counts['resolver_error_count'] += 1
                self._event(self.fault_reason,now,entry)
                break
            try:
                commit(entry.payload, transform, entry.source_stamp_s)
            except Exception as exc:
                self.fault_reason = 'observation_commit_error:'+type(exc).__name__
                self.counts['commit_error_count'] += 1
                self._event(self.fault_reason,now,entry)
                break
            self.pending.popleft()
            self.history_stamps_ns.append(entry.stamp_ns)
            self.history_source_stamps_s.append(entry.source_stamp_s)
            self.counts['committed_count'] += 1
            self.commit_audit.append({'stamp_s':entry.source_stamp_s,'committed_at_s':now/1e9,
                                     'age_at_commit_s':(now-entry.stamp_ns)/1e9,
                                     'queue_residence_s':(now-entry.received_ns)/1e9,
                                     'tf_wait_s':0.0 if entry.first_tf_wait_ns is None else (now-entry.first_tf_wait_ns)/1e9,
                                     'tf_attempts':entry.tf_attempts})
            committed += 1
        return committed

    @property
    def terminal_failure_count(self):
        return sum(self.counts[name] for name in ('expired_count','invalid_stamp_count','out_of_order_count',
                    'overflow_count','future_stamp_rejected_count','resolver_error_count','commit_error_count','clock_reset_count'))

    def health(self, now_s):
        now = self.observe_time(now_s)
        latest = self.history_stamps_ns[-1] if self.history_stamps_ns else None
        age = None if latest is None else (now-latest)/1e9
        reasons = []
        if self.fault_reason:
            reasons.append(self.fault_reason)
        if len(self.history_stamps_ns) < self.HISTORY_SIZE:
            reasons.append('six_real_history_samples_not_ready')
        if age is None or age < 0:
            reasons.append('committed_scan_age_invalid')
        elif now-latest > self.MAXIMUM_AGE_NS:
            reasons.append('committed_scan_stale')
        return {'healthy':not reasons,'failures':reasons,'latest_committed_scan_age_s':age,
                'real_history_count':len(self.history_stamps_ns)}

    def report(self):
        waits = [row['tf_wait_s'] for row in self.commit_audit if row['tf_wait_s'] > 0]
        return {'contract':{'maximum_age_s':.20,'source_stamp_sample_interval_s':.20,
                             'history_size':6,'maximum_pending':8,'lookup_semantics':'exact positive scan timestamp only',
                             'sampling_operator':'inherited floating source-stamp difference >= 0.20'},
                'counts':dict(self.counts),'terminal_failure_count':self.terminal_failure_count,
                'pending_count':len(self.pending),'maximum_queue_depth':self.maximum_queue_depth,
                'history_stamps_s':[s/1e9 for s in self.history_stamps_ns],
                'maximum_completed_tf_wait_s':max(waits) if waits else 0.0,
                'mean_completed_tf_wait_s':sum(waits)/len(waits) if waits else 0.0,
                'fault_reason':self.fault_reason,'events':list(self.events),'commits':list(self.commit_audit)}
