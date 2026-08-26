# Regression tests for open-rmf/rmf_ros2#539:
# broken def_property setters that accepted only `self` and raised TypeError
# on every Python assignment.

import rmf_adapter as adpt
import rmf_adapter.fleet_update_handle as fuh


def _setter_accepts_value(prop) -> bool:
    """True if the pybind property setter takes a value argument (self + value).

    Broken setters in this package expose docs like:
      (arg0: SomeType) -> Optional[datetime.timedelta]
    Fixed setters look like:
      (arg0: SomeType, arg1: Optional[datetime.timedelta]) -> None
    """
    assert prop.fset is not None, "property has no setter"
    doc = prop.fset.__doc__ or ""
    # Count comma-separated parameters inside the leading (...) signature.
    open_paren = doc.find("(")
    close_paren = doc.find(")", open_paren + 1)
    assert open_paren >= 0 and close_paren > open_paren, (
        f"unexpected setter docstring: {doc!r}")
    params = doc[open_paren + 1:close_paren].strip()
    if not params:
        return False
    return params.count(",") >= 1


def test_maximum_delay_setter_accepts_value():
    assert _setter_accepts_value(adpt.RobotUpdateHandle.maximum_delay)


def test_default_maximum_delay_setter_accepts_value():
    assert _setter_accepts_value(adpt.FleetUpdateHandle.default_maximum_delay)


def test_confirmation_errors_property_roundtrip():
    """Confirmation is default-constructible; exercise the fixed errors setter."""
    confirm = fuh.Confirmation()
    confirm.errors = ["first", "second"]
    assert list(confirm.errors) == ["first", "second"]
