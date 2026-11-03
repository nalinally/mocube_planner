from mocube_planner.commands import EDGE_LENGTH
from mocube_planner.executor import assign_ids, before_step, classify_position, load_plan
import pytest

# ID0 の +y 側にある 2 台のうち ID1 を +x へ 1 マス。支えは ID2
PLAN = {
    'edge_length_mm': 113.1,
    'speed_gain_mm': 0.2,
    'initial': {'0': [0, 0, 0], '1': [0, 1, 0], '2': [1, 0, 0]},
    'steps': [{
        'module': 1, 'from': [0, 1, 0], 'to': [1, 1, 0], 'duration_s': 1.0,
        'commands': [{'module': 0, 'servo': 2, 'face': 'f', 'angle_deg': 100.0}],
        'positions': {'0': [0, 0, 0], '1': [1, 1, 0], '2': [1, 0, 0]},
    }],
}
START = {0: (0, 0, 0), 1: (0, 1, 0), 2: (1, 0, 0)}
END = {0: (0, 0, 0), 1: (1, 1, 0), 2: (1, 0, 0)}


def meters(positions):
    return {i: tuple(v * EDGE_LENGTH for v in c) for i, c in positions.items()}


def test_assign_ids_maps_plan_ids_to_actual_ids_by_cell():
    plan = assign_ids(load_plan(PLAN), meters({0: (0, 0, 0), 7: (0, 1, 0), 3: (1, 0, 0)}),
                      EDGE_LENGTH, 0.01)
    assert plan['steps'][0]['module'] == 7
    assert plan['steps'][0]['positions'][7] == (1, 1, 0)
    assert plan['edge_length_mm'] == 113.1


def test_assign_ids_refuses_different_shape():
    with pytest.raises(ValueError, match='初期形状'):
        assign_ids(load_plan(PLAN), meters({0: (0, 0, 0), 1: (0, 1, 0), 2: (2, 0, 0)}),
                   EDGE_LENGTH, 0.01)


def test_before_step_is_initial_then_previous_positions():
    plan = load_plan(PLAN)
    assert before_step(plan, 0) == plan['initial']
    assert before_step(plan, 1) == plan['steps'][0]['positions']


@pytest.mark.parametrize('positions, expected', [
    (START, 'before'),
    (END, 'after'),
    ({**START, 1: (0.5, 1, 0)}, 'unknown'),
    ({0: (0, 0, 0)}, 'unknown'),
])
def test_classify_position(positions, expected):
    assert classify_position(meters(positions), START, END, EDGE_LENGTH, 0.01) == expected


def test_load_plan_rejects_out_of_range_angle():
    plan = {**PLAN, 'steps': [{
        **PLAN['steps'][0],
        'commands': [{'module': 0, 'servo': 2, 'face': 'f', 'angle_deg': 200.0}],
    }]}
    with pytest.raises(ValueError, match='範囲外'):
        load_plan(plan)


def test_load_plan_rejects_plan_without_settings():
    plan = {k: v for k, v in PLAN.items() if k != 'edge_length_mm'}
    with pytest.raises(ValueError, match='作り直して'):
        load_plan(plan)
