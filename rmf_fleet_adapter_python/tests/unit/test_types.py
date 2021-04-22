import rmf_adapter.type as types
import numpy as np
import datetime


# TYPES ======================================================================
def test_types():
    # Test CPPDeliveryMsg
    msg = types.CPPDeliveryMsg("pickup_place",
                               "pickup_dispenser",
                               "dropoff_place",
                               "dropoff_ingestor")
    assert msg.pickup_place_name == "pickup_place"
    assert msg.pickup_dispenser == "pickup_dispenser"
    assert msg.dropoff_place_name == "dropoff_place"
    assert msg.dropoff_ingestor == "dropoff_ingestor"

    msg.pickup_place_name += "_rawr"
    msg.pickup_dispenser += "_rawr"
    msg.dropoff_place_name += "_rawr"
    msg.dropoff_ingestor += "_rawr"

    assert msg.pickup_place_name == "pickup_place_rawr"
    assert msg.pickup_dispenser == "pickup_dispenser_rawr"
    assert msg.dropoff_place_name == "dropoff_place_rawr"
    assert msg.dropoff_ingestor == "dropoff_ingestor_rawr"
