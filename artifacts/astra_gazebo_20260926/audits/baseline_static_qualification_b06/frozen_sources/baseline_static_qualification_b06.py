"""B06 zero-authority typed-seed repair; frozen B05 math and proposal unchanged.

This is a new adapter, not a B06 execution stage or release. Valid calls return
B05's existing unallocated proposal, including its original draft identities.
No runtime defaults, physical geometry, receipt or ROS implementation is added.
"""
import baseline_static_qualification_b05 as _b05


def proposal_schedule(scene_ids, historical_rows, other_proposed_seeds=()):
    """Require a list/tuple of exact built-in ints; no coercion or filtering.

    JSON-array lists and the default tuple are supported. Strings, bytes,
    mappings and one-shot/lazy iterables are rejected instead of interpreted
    as seed containers. Bool, float and int subclasses are not exact ints.
    Freeze a copy before delegating so only the checked values reach B05.
    """
    if type(other_proposed_seeds) not in (list, tuple):
        raise ValueError('other_proposed_seeds requires a list or tuple of exact ints')
    checked = tuple(other_proposed_seeds)
    if any(type(seed) is not int for seed in checked):
        raise ValueError('other_proposed_seeds contains a non-exact-int seed')
    return _b05.proposal_schedule(scene_ids, historical_rows, checked)
