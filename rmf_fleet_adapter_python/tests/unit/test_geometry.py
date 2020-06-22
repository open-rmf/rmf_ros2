import rmf_adapter.geometry as geometry


# CIRCLE ======================================================================
# Check construction
def test_circle():
    circle = geometry.Circle(5)
    circle_copy = geometry.Circle(circle)  # Copy construction works!
    assert circle.radius == 5
    assert circle_copy.radius == 5

    # Check member reassignment
    circle.radius = 10
    circle_copy.radius = 10
    assert circle.radius == 10
    assert circle_copy.radius == 10

    # Check direct construction
    direct_final_convex_circle = geometry.make_final_convex_circle(5)
    direct_final_convex_circle_source = direct_final_convex_circle.source
    assert direct_final_convex_circle.characteristic_length == 5
    assert direct_final_convex_circle_source.radius == 5

    # Check method construction
    final_circle = circle.finalize()
    final_convex_circle = circle.finalize_convex()
    final_circle_source = final_circle.source

    # Verify that source is a copy
    # and does not affect the final shape it was generated from
    assert final_circle.characteristic_length == 10.0
    final_circle_source.radius = 1.0
    assert final_circle_source.radius == 1.0
    assert final_circle.characteristic_length == 10.0

    assert final_convex_circle.characteristic_length == 10
    assert final_convex_circle.source.radius == 10
