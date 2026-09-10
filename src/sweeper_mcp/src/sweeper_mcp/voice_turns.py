"""Bounded, clock-driven text assembly; no ROS, cloud, or control capability."""
import math
import re


def compact(text):
    return "".join(c for c in str(text) if c.isalnum())


SHORT_COMMANDS = {"停止", "停", "取消", "暂停", "继续", "恢复", "停下", "停止清扫", "取消任务"}
ACKS = {"是", "对", "好的", "好", "可以", "不是", "不对"}
END_MARKERS = {"说完了", "我说完了", "指令说完了"}
ACTION = re.compile(r"清扫|打扫|扫[ABCabc区]|导航|返回|回到|前进|后退|左转|右转|移动|查询|查看|看一下|看下|有没有|停止|取消|暂停|继续|恢复|跳过|去|到")
UNFINISHED = re.compile(r"(?:然后|再|再到|再去|先去|先|到|去|在|之后|以后|接着|如果|如果有|的话|帮我|帮我把|请你|把|以及|还有|不是|而是)$")


def text_kind(text, awaiting_reply=False):
    value = compact(text)
    if not value or set(value) <= set("嗯呃啊额唔哦噢"):
        return "ignore"
    if value in END_MARKERS:
        return "end"
    if value in SHORT_COMMANDS:
        return "command"
    if value in ACKS:
        return "reply" if awaiting_reply else "fragment"
    value = value.rstrip("嗯呃啊额唔哦噢")
    if UNFINISHED.search(value):
        return "fragment"
    if ACTION.search(value):
        return "complete"
    return "reply" if awaiting_reply else "fragment"


class VoiceTurnBuffer:
    def __init__(self, quiet_s=1.5, incomplete_s=4.0, fragment_ttl_s=6.0,
                 max_age_s=60.0, max_chars=4000):
        values = (quiet_s, incomplete_s, fragment_ttl_s, max_age_s)
        if not all(math.isfinite(float(x)) and float(x) > 0 for x in values):
            raise ValueError("voice turn timeouts must be finite and positive")
        if not quiet_s <= incomplete_s <= fragment_ttl_s <= max_age_s:
            raise ValueError("voice turn timeout order is invalid")
        self.quiet_s, self.incomplete_s = float(quiet_s), float(incomplete_s)
        self.fragment_ttl_s, self.max_age_s = float(fragment_ttl_s), float(max_age_s)
        self.max_chars = int(max_chars)
        self.clear()

    def clear(self):
        self.parts = []
        self.started = self.last_end = self.updated = 0.0
        self.force = False
        self.reply = False

    @property
    def text(self):
        return "，".join(self.parts)

    def add(self, text, ended_at, now, awaiting_reply=False):
        kind = text_kind(text, awaiting_reply)
        if kind == "ignore":
            return "IGNORED"
        if self.parts and now - self.started > self.max_age_s:
            self.clear()
        if (self.parts and not self.reply and not ACTION.search(compact(self.text)) and
                float(ended_at) - self.last_end >= self.fragment_ttl_s):
            self.clear()
        if kind == "end":
            self.force = bool(self.parts)
            return "END" if self.force else "IGNORED"
        # Stop/cancel must not become a suffix of a pending movement request.
        if kind == "command":
            self.clear()
        if len(self.text) + len(text) + 1 > self.max_chars:
            self.clear()
            return "OVERFLOW"
        if not self.parts:
            self.started = now
        self.parts.append(str(text).strip())
        self.last_end, self.updated = float(ended_at), float(now)
        self.reply = self.reply or awaiting_reply
        self.force = kind == "command"
        return "WAITING" if kind == "fragment" else "BUFFERING"

    def poll(self, now, last_speech_at=0.0, busy=False):
        if not self.parts:
            return None
        age = now - self.started
        if age > self.max_age_s:
            self.clear()
            return {"state": "EXPIRED", "text": ""}
        if busy:
            return None
        quiet = now - max(self.last_end, last_speech_at)
        kind = text_kind(self.text, self.reply)
        meaningful = bool(ACTION.search(compact(self.text))) or self.reply
        if not meaningful and not self.force:
            if now - self.last_end >= self.fragment_ttl_s:
                self.clear()
                return {"state": "EXPIRED", "text": ""}
            return None
        delay = self.incomplete_s if kind == "fragment" else self.quiet_s
        if not self.force and (quiet < delay or now - self.updated < 0.25):
            return None
        value = self.text
        self.clear()
        return {"state": "READY", "text": value}
