import rmf_adapter as adpt


def test_robot_update_handle_maximum_delay_setter_accepts_value():
    doc = adpt.RobotUpdateHandle.maximum_delay.fset.__doc__
    assert "arg1" in doc, doc


def test_fleet_update_handle_default_maximum_delay_setter_accepts_value():
    doc = adpt.FleetUpdateHandle.default_maximum_delay.fset.__doc__
    assert "arg1" in doc, doc
