from v4_scope_rl.evaluation import rollout


def test_analytic_replay_is_exact():
    first = rollout("analytic", 901, record_trace=True)
    second = rollout("analytic", 901, record_trace=True)
    assert first == second
    assert first["trace_sha256"] == second["trace_sha256"]
