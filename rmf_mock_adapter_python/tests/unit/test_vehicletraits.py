from typing import Callable
import datetime


def init():  # Hacky way to avoid flake8 errors
    import os
    import numpy as np

    try:
        rmf_adapter_path = ("~/Desktop/"
                            "pybind_ws/install/rmf_mock_adapter_python/"
                            "lib/python3.6/site-packages")
        os.chdir(os.path.expanduser(rmf_adapter_path))
    except Exception:
        pass

    import rmf_adapter as adpt
    import rmf_adapter.vehicletraits as traits
    import rmf_adapter.geometry as geometry

    global adpt, traits, geometry, np


init()

# LIMITS ======================================================================
# Check instantiation and getters
limits = traits.Limits()
assert limits.nominal_velocity == 0.0
assert limits.nominal_acceleration == 0.0
assert not limits.valid  # False when not positive

# Check setters
limits.nominal_velocity = 1.0
limits.nominal_acceleration = 1.0
assert limits.nominal_velocity == 1.0
assert limits.nominal_acceleration == 1.0
assert limits.valid

# STEERING ====================================================================
traits.Steering.Differential
traits.Steering.Holonomic

# DIFFERENTIAL ================================================================
# Check instantiation and getters
differential = traits.Differential()
assert all(differential.forward == np.array([1, 0]))
assert differential.reversible
assert differential.valid

# Check setters
differential.forward = np.array([0., 0])
differential.reversible = False
assert all(differential.forward == np.array([0, 0]))
assert not differential.reversible
assert not differential.valid  # Forward length is now too close to 0

# HOLONOMIC ===================================================================
holonomic = traits.Holonomic()

# PROFILE =====================================================================
# Setup
circle = geometry.make_final_convex_circle(5)
big_circle = geometry.make_final_convex_circle(10)

# Check instantiation
profile = traits.Profile(circle)
profile_with_vicinity = traits.Profile(big_circle, circle)

# Check getters
assert profile.footprint.characteristic_length == circle.characteristic_length
assert profile.vicinity.characteristic_length == circle.characteristic_length

assert profile_with_vicinity.footprint.characteristic_length == \
    big_circle.characteristic_length
assert profile_with_vicinity.vicinity.characteristic_length == \
    circle.characteristic_length

# Check setters
final_circle = geometry.make_final_convex_circle(20)
profile.footprint = final_circle
profile_with_vicinity.footprint = final_circle

# Check tha vicinity is overwritten
# This particular profile was passed a nullptr for vicinity!
assert profile.footprint.characteristic_length == \
    final_circle.characteristic_length
assert profile.vicinity.characteristic_length == \
    final_circle.characteristic_length

# Check if setting vicinity decouples the two
profile.vicinity = circle
profile.footprint = big_circle

assert profile.footprint.characteristic_length == \
    big_circle.characteristic_length
assert profile.vicinity.characteristic_length == \
    circle.characteristic_length


# Check that vicinity is not overwritten
assert profile_with_vicinity.footprint.characteristic_length == \
    final_circle.characteristic_length
assert profile_with_vicinity.vicinity.characteristic_length == \
    circle.characteristic_length


# VEHICLETRAITS ===============================================================
vel_limits = traits.Limits(1, 1)
ang_limits = traits.Limits(2, 2)

# Check instantiation
valid_traits = traits.VehicleTraits(vel_limits, ang_limits, profile)
invalid_traits = traits.VehicleTraits(
    vel_limits, ang_limits, profile, differential
)

# Check validity
assert valid_traits.valid
assert not invalid_traits.valid  # Differential passed is invalid

# Check getters
assert valid_traits.linear.nominal_velocity == 1
assert valid_traits.linear.nominal_acceleration == 1
assert valid_traits.rotational.nominal_velocity == 2
assert valid_traits.rotational.nominal_acceleration == 2
assert valid_traits.profile.footprint.characteristic_length == 10
assert valid_traits.profile.footprint.characteristic_length == 10
assert valid_traits.steering == traits.Steering.Differential
assert all(valid_traits.differential.forward == np.array([1, 0]))
assert valid_traits.differential.reversible
assert valid_traits.differential.valid
assert not valid_traits.holonomic  # No Implementation

# Quirk of bindings
assert valid_traits.differential is valid_traits.const_differential
