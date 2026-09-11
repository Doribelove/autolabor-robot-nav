"""Behavioral checks for pauses, delayed recognition, fragments and resets."""
from sweeper_mcp.voice_turns import VoiceTurnBuffer, text_kind


def test_natural_pauses_preserve_order_and_unfinished_suffix():
    b = VoiceTurnBuffer()
    b.add("先清扫A区然后", 1, 1)
    assert b.poll(3.5) is None
    b.add("再到", 3.6, 3.6)
    assert b.poll(5) is None
    b.add("研究院大楼前大厅清扫", 5.1, 5.1)
    assert b.poll(6) is None
    assert b.poll(8) is None
    assert b.poll(9.2)["text"] == "先清扫A区然后，再到，研究院大楼前大厅清扫"
    assert b.poll(12) is None


def test_short_fragments_expire_but_fillers_never_start_a_turn():
    b = VoiceTurnBuffer()
    for text in ("嗯", "呃", "！！！", "嗯嗯"):
        assert b.add(text, 1, 1) == "IGNORED"
    assert not b.parts
    b.add("A区", 2, 2)
    assert b.poll(4) is None
    assert b.poll(8.1)["state"] == "EXPIRED"
    b.add("清扫B区", 9, 9)
    assert b.poll(11) is None
    assert b.poll(12.1)["text"] == "清扫B区"


def test_delayed_asr_and_speech_activity_prevent_early_dispatch():
    b = VoiceTurnBuffer()
    b.add("清扫A区", 1, 6)
    assert b.poll(6.1) is None  # callback settling grace
    assert b.poll(8, last_speech_at=7.5, busy=True) is None
    b.add("然后清扫B区", 8, 10)
    assert b.poll(10.3) is None
    assert b.poll(12.1)["text"] == "清扫A区，然后清扫B区"


def test_stop_is_valid_and_replaces_unsubmitted_draft():
    b = VoiceTurnBuffer()
    b.add("先清扫A区然后", 1, 1)
    b.add("停止", 2, 2)
    assert b.poll(2)["text"] == "停止"
    for value in ("停止", "取消", "暂停", "恢复"):
        assert text_kind(value) == "command"


def test_contextual_region_reply_and_explicit_end():
    b = VoiceTurnBuffer()
    b.add("前大厅", 1, 1, awaiting_reply=True)
    assert b.poll(3)["text"] == "前大厅"
    b.add("先清扫A区", 4, 4)
    b.add("说完了", 4.2, 4.2)
    assert b.poll(4.2)["text"] == "先清扫A区"


def test_capacity_and_session_clear_do_not_leak_old_text():
    b = VoiceTurnBuffer(max_chars=10)
    b.add("先清扫A区", 1, 1)
    assert b.add("然后清扫研究院大楼前大厅", 2, 2) == "OVERFLOW"
    assert not b.parts
    b.add("清扫B区", 3, 3)
    b.clear()
    assert b.poll(100) is None


def test_filler_does_not_keep_a_fragment_alive():
    b = VoiceTurnBuffer()
    b.add("那里", 1, 1)
    b.add("嗯", 6, 6)
    assert b.poll(7.1, last_speech_at=6)["state"] == "EXPIRED"


def test_expired_orphan_is_not_spliced_when_new_audio_was_busy():
    b = VoiceTurnBuffer()
    b.add("A区", 1, 1)
    assert b.poll(9, busy=True) is None
    b.add("清扫B区", 10, 10)
    assert b.poll(13.1)["text"] == "清扫B区"


def test_pause_matrix_and_fillers_after_conjunction():
    for pause in (0.3, 0.6, 1.0, 1.5, 2.5):
        b = VoiceTurnBuffer()
        b.add("清扫A区然后呃", 1, 1)
        assert b.poll(1 + pause) is None
        b.add("再清扫B区", 1 + pause, 1 + pause)
        assert b.poll(3 + pause) is None
        assert b.poll(5.1 + pause)["text"] == "清扫A区然后呃，再清扫B区"


def test_complete_motion_clause_keeps_natural_pause_and_same_turn():
    for prefix in ("清扫A区", "先清扫A区", "请先清扫A区"):
        for pause in (1.6, 2.5, 2.9):
            b = VoiceTurnBuffer()
            b.add(prefix, 10, 10)
            assert b.poll(10 + pause) is None
            b.add("再回到当前点", 11 + pause, 11 + pause)
            assert b.poll(13 + pause) is None
            b.add("进行视觉清扫", 14 + pause, 14 + pause)
            assert b.poll(18.1 + pause)["text"] == prefix + "，再回到当前点，进行视觉清扫"


def test_waiting_is_bounded_and_explicit_end_or_stop_is_immediate():
    b = VoiceTurnBuffer()
    b.add("先清扫A区", 1, 1)
    assert b.poll(4.9) is None
    assert b.poll(5.1)["text"] == "先清扫A区"
    for ending in ("说完了", "停止", "取消"):
        b.add("先清扫A区", 10, 10)
        b.add(ending, 10.1, 10.1)
        assert b.poll(10.1)["text"] == ("先清扫A区" if ending == "说完了" else ending)


def test_query_reply_delay_stays_short():
    b = VoiceTurnBuffer()
    b.add("查询电量", 1, 1)
    assert b.poll(2.6)["text"] == "查询电量"
