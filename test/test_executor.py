from mocube_planner.commands import EDGE_LENGTH
from mocube_planner.executor import (
    assign_ids,
    expected_receptions,
    load_plan,
    Observation,
    PlanRunner,
    Status,
)
import pytest

# ID0 の +y 側にある 2 台のうち ID1 を +x へ 1 マス。支えは ID2
PLAN = {
    'initial': {'0': [0, 0, 0], '1': [0, 1, 0], '2': [1, 0, 0]},
    'steps': [{
        'module': 1, 'from': [0, 1, 0], 'to': [1, 1, 0], 'duration_s': 1.0,
        'commands': [{'module': 0, 'servo': 2, 'face': 'f', 'angle_deg': 100.0}],
        'positions': {'0': [0, 0, 0], '1': [1, 1, 0], '2': [1, 0, 0]},
    }],
}


def meters(positions):
    return {i: tuple(v * EDGE_LENGTH for v in c) for i, c in positions.items()}


def observe(positions, receptions=None, online=(0, 1, 2)):
    return Observation(set(online), meters(positions), receptions or {})


START = {0: (0, 0, 0), 1: (0, 1, 0), 2: (1, 0, 0)}
END = {0: (0, 0, 0), 1: (1, 1, 0), 2: (1, 0, 0)}


def runner(require_ir=False):
    return PlanRunner(load_plan(PLAN), EDGE_LENGTH, 0.01, 1.0, 1.0, require_ir)


def test_runs_plan_to_done():
    r = runner()
    assert r.update(100.0, observe(START)) == [(0, 2, 100.0)]
    assert r.status == Status.DRIVING
    assert r.update(100.5, observe(START)) == []
    assert r.update(101.0, observe(END)) == [(0, 2, 0.0)]
    assert r.status == Status.VERIFYING
    assert r.update(101.1, observe(END)) == []
    assert r.status == Status.DONE


def test_fails_when_position_stays_wrong():
    r = runner()
    r.update(100.0, observe(START))
    r.update(101.0, observe(START))
    r.update(101.5, observe(START))
    assert r.status == Status.VERIFYING
    r.update(102.5, observe(START))
    assert r.status == Status.FAILED
    assert '1 手目' in r.error


def test_stops_servos_when_module_goes_offline():
    r = runner()
    r.update(100.0, observe(START))
    assert r.update(100.5, observe(START, online=(1, 2))) == [(0, 2, 0.0)]
    assert r.status == Status.FAILED


def test_requires_ir_for_new_contact_received_after_step_start():
    r = runner(require_ir=True)
    r.update(100.0, observe(START))
    r.update(101.0, observe(END))
    old = {(2, 'f'): (1, 99.0)}
    r.update(101.1, observe(END, old))
    assert r.status == Status.VERIFYING
    r.update(101.2, observe(END, {(2, 'f'): (1, 101.1)}))
    assert r.status == Status.DONE


def test_expected_receptions_only_for_new_contacts():
    assert expected_receptions(START, END) == {(2, 'f', 1)}


def test_assign_ids_maps_plan_ids_to_actual_ids_by_cell():
    plan = assign_ids(load_plan(PLAN), meters({0: (0, 0, 0), 7: (0, 1, 0), 3: (1, 0, 0)}),
                      EDGE_LENGTH, 0.01)
    assert plan['steps'][0]['module'] == 7
    assert plan['steps'][0]['positions'][7] == (1, 1, 0)


def test_refuses_to_start_on_different_shape():
    r = runner()
    assert r.update(100.0, observe({0: (0, 0, 0), 1: (0, 1, 0), 2: (2, 0, 0)})) == []
    assert r.status == Status.FAILED
    assert '初期形状' in r.error


def test_fails_when_modules_never_appear():
    r = runner()
    r.update(100.0, Observation())
    assert r.status == Status.WAITING
    r.update(102.0, Observation())
    assert r.status == Status.FAILED


def test_load_plan_rejects_out_of_range_angle():
    plan = {**PLAN, 'steps': [{
        **PLAN['steps'][0],
        'commands': [{'module': 0, 'servo': 2, 'face': 'f', 'angle_deg': 200.0}],
    }]}
    with pytest.raises(ValueError, match='範囲外'):
        load_plan(plan)
