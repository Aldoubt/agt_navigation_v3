from agt_nav_benchmark.localization_shadow import (
    REPLAY_REQUIRED_TOPICS,
    evaluate_replay_contract,
)


def _valid_types():
    return dict(REPLAY_REQUIRED_TOPICS)


def _valid_counts():
    return {topic: 1 for topic in REPLAY_REQUIRED_TOPICS}


def test_complete_replay_contract_is_ok():
    result = evaluate_replay_contract(_valid_types(), _valid_counts())
    assert result.status == 'OK'
    assert result.ok


def test_missing_topic_blocks_contract():
    types = _valid_types()
    types.pop('/agt/relocalization/pose')
    result = evaluate_replay_contract(types, _valid_counts())
    assert result.status == 'BLOCKED_INPUT_CONTRACT'
    assert result.missing_topics == ('/agt/relocalization/pose',)


def test_zero_message_topic_blocks_contract():
    counts = _valid_counts()
    counts['/agt/map_tracking/pose'] = 0
    result = evaluate_replay_contract(_valid_types(), counts)
    assert result.status == 'BLOCKED_INPUT_CONTRACT'
    assert result.zero_message_topics == ('/agt/map_tracking/pose',)


def test_type_mismatch_blocks_contract():
    types = _valid_types()
    types['/agt/localization/status'] = 'std_msgs/msg/String'
    result = evaluate_replay_contract(types, _valid_counts())
    assert result.status == 'BLOCKED_INPUT_CONTRACT'
    assert result.type_mismatches == ((
        '/agt/localization/status',
        'agt_robot_interfaces/msg/LocalizationStatus',
        'std_msgs/msg/String',
    ),)
