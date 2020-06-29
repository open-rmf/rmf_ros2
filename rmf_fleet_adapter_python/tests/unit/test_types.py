import rmf_adapter.type as types
import numpy as np
import datetime


# TYPES ======================================================================
def test_types():
    # Test nullopt
    assert types.NullOptional()

    # Test CPPDeliveryMsg
    msg = types.CPPDeliveryMsg("task",
                               "pickup_place",
                               "pickup_dispenser",
                               "dropoff_place",
                               "dropoff_dispenser")
    assert msg.task_id == "task"
    assert msg.pickup_place_name == "pickup_place"
    assert msg.pickup_dispenser == "pickup_dispenser"
    assert msg.dropoff_place_name == "dropoff_place"
    assert msg.dropoff_dispenser == "dropoff_dispenser"

    msg.task_id += "_rawr"
    msg.pickup_place_name += "_rawr"
    msg.pickup_dispenser += "_rawr"
    msg.dropoff_place_name += "_rawr"
    msg.dropoff_dispenser += "_rawr"

    assert msg.task_id == "task_rawr"
    assert msg.pickup_place_name == "pickup_place_rawr"
    assert msg.pickup_dispenser == "pickup_dispenser_rawr"
    assert msg.dropoff_place_name == "dropoff_place_rawr"
    assert msg.dropoff_dispenser == "dropoff_dispenser_rawr"

    # Test optionals
    assert types.OptionalULong(1).value == 1

    assert types.OptionalDouble(1).value == 1
    assert types.OptionalDouble(1.0).value == 1

    assert np.ndarray.all(
        types.OptionalVector2D([1, 2]).value == np.array([1, 2])
    )

    assert types.OptionalDuration(datetime.timedelta(seconds=60))
