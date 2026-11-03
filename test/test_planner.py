import json
from pathlib import Path

from mocube_planner.backends.pathfinder import parse_scen, PathfinderBackend, to_module_moves
from mocube_planner.commands import build_plan, EDGE_LENGTH, replay, servo_commands
from mocube_planner.model import check_move, contacts, load_scenario, unreachable_reason
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
INITIAL = PACKAGE / 'scenarios' / 'l_wall_to_floor_initial.json'
FINAL = PACKAGE / 'scenarios' / 'l_wall_to_floor_final.json'
SCEN = PACKAGE / 'results' / 'l_wall_to_floor.scen'
GOLDEN = PACKAGE / 'results' / 'l_wall_to_floor_commands.json'


def l_wall_moves(initial):
    cells, moves = parse_scen(SCEN.read_text(encoding='utf-8'))
    return to_module_moves(initial, cells, moves)


def normalized(plan):
    """JSON の読み書きで変わる tuple と数値キーを揃える."""
    return json.loads(json.dumps(plan))


def test_l_wall_plan_matches_previous_result():
    initial = load_scenario(INITIAL)
    goal = set(load_scenario(FINAL).values())
    plan = build_plan(initial, l_wall_moves(initial), goal)
    assert len(plan['steps']) == 10
    assert normalized(plan) == json.loads(GOLDEN.read_text(encoding='utf-8'))
    assert replay(plan) < 0.001


def test_build_plan_rejects_wrong_goal():
    initial = load_scenario(INITIAL)
    with pytest.raises(ValueError, match='目標形状'):
        build_plan(initial, l_wall_moves(initial), set(initial.values()))


@pytest.mark.parametrize('positions, module, step, message', [
    ({0: (0, 0, 0), 1: (1, 0, 0)}, 0, (0, 1, 0), 'ID0'),
    ({0: (0, 0, 0), 1: (1, 0, 0), 2: (2, 0, 0)}, 1, (1, 0, 0), 'ふさがって'),
    ({0: (0, 0, 0), 1: (1, 0, 0)}, 1, (0, 1, 0), '支え'),
    ({0: (0, 0, 0), 1: (1, 0, 0), 2: (0, 1, 0), 3: (1, 1, 0)}, 1, (1, 1, 0), '1 軸 1 マス'),
])
def test_check_move_rejects(positions, module, step, message):
    with pytest.raises(ValueError, match=message):
        check_move(positions, module, step)


def test_contacts_order_by_axis():
    assert contacts({0: (0, 0, 0), 1: (1, 0, 0), 2: (0, 0, -1)}) == {(0, 1, 0), (2, 0, 2)}


def test_block_to_chair_is_unreachable_one_module_at_a_time():
    initial = load_scenario(PACKAGE / 'scenarios' / 'block_to_chair_initial.json')
    goal = set(load_scenario(PACKAGE / 'scenarios' / 'block_to_chair_final.json').values())
    assert 'y 軸' in unreachable_reason(initial, goal) or 'x 軸' in unreachable_reason(initial, goal)


def test_l_wall_is_not_flagged_unreachable():
    initial = load_scenario(INITIAL)
    assert unreachable_reason(initial, set(load_scenario(FINAL).values())) is None


def test_edge_length_matches_estimator_default():
    assert EDGE_LENGTH == pytest.approx(0.1131)


def test_parse_scen_ignores_color_header():
    text = ('0, 255, 255, 255, 90\n1, 255, 255, 255, 90\n\n'
            '00, 0, 1, 2, 3\n01, 1, 4, 5, 6\n\n*00, 0, 0, 1, 0\n')
    assert parse_scen(text) == ({0: (1, 2, 3), 1: (4, 5, 6)}, [(0, (0, 1, 0))])


def test_backend_reports_missing_binary(tmp_path):
    with pytest.raises(FileNotFoundError, match='build_pathfinder'):
        PathfinderBackend(tmp_path / 'Pathfinder', PACKAGE / 'moves')


def test_servo_commands_cover_contact_beyond_the_destination():
    # ID1 が ID0, ID2 の +y 側を +x へ。ID3 は行き過ぎたときにだけ接する
    positions = {0: (0, 0, 0), 1: (0, 1, 0), 2: (1, 0, 0), 3: (2, 0, 0)}
    assert (3, 5) not in servo_commands({i: c for i, c in positions.items() if i != 3},
                                        1, (1, 0, 0), 100.0)
    assert (1, 5) in servo_commands(positions, 1, (1, 0, 0), 100.0)
    # ID1 が -y 側のときは、相手の b 面のサーボを回す
    positions = {0: (0, 0, 0), 1: (0, -1, 0), 2: (1, 0, 0), 3: (2, 0, 0)}
    assert (3, 5) in servo_commands(positions, 1, (1, 0, 0), 100.0)


def test_servo_commands_do_not_push_a_module_two_cells_ahead():
    positions = {0: (0, 0, 0), 1: (0, -1, 0), 2: (1, 0, 0), 3: (2, 0, 0), 4: (2, -1, 0)}
    assert (3, 5) not in servo_commands(positions, 1, (1, 0, 0), 100.0)
