from app.main import parse_param_file


def test_formatos_mission_planner():
    txt = "# comentario\nARMING_CHECK,-5\nCOMPASS_USE 0\nRC1_MIN\t1058\nlixo\n"
    assert parse_param_file(txt) == {"ARMING_CHECK": -5.0, "COMPASS_USE": 0.0, "RC1_MIN": 1058.0}
