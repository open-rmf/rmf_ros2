#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include <cstdint>
#include "RingBuffer.hpp"

using namespace rmf_websocket;

TEST_CASE("RingBuffer pops item in correct order", "[RingBuffer]") {
  RingBuffer<int> buffer(10);

  REQUIRE(buffer.pop_item() == std::nullopt);

  buffer.push(1);
  buffer.push(2);
  buffer.push(3);

  REQUIRE(buffer.pop_item().value() == 1);
  REQUIRE(buffer.pop_item().value() == 2);
  REQUIRE(buffer.pop_item().value() == 3);
  REQUIRE(buffer.pop_item() == std::nullopt);
}


TEST_CASE("RingBuffer overwrites old values in correct order", "[RingBuffer]") {
  RingBuffer<int> buffer(2);

  REQUIRE(buffer.pop_item() == std::nullopt);

  buffer.push(1);
  buffer.push(2);
  buffer.push(3);
  REQUIRE(buffer.pop_item().value() == 2);
  REQUIRE(buffer.pop_item().value() == 3);
  REQUIRE(buffer.pop_item() == std::nullopt);
}
