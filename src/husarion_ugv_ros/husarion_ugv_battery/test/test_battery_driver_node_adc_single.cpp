// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utils/test_battery_driver_node.hpp"

#include <chrono>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_utils/test/ros_test_utils.hpp"

class TestBatteryNodeADCSingle : public TestBatteryNode
{
public:
  TestBatteryNodeADCSingle() : TestBatteryNode(true, false) {}
};

TEST_F(TestBatteryNodeADCSingle, BatteryValues)
{
  // Change some values for ADC channels used by second battery.
  // As this is test for single battery this should not affect published battery state.
  WriteNumberToFile<int>(1600, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
  WriteNumberToFile<int>(100, std::filesystem::path(device0_path_ / "in_voltage2_raw"));

  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // This is done to check if channels of ADC readers were assigned correctly, not to verify
  // calculations. If any test performing calculations fails this test will most likely fail too.
  EXPECT_FLOAT_EQ(35.05957, battery_state_->voltage);
  EXPECT_FLOAT_EQ(2.01, battery_state_->current);
  EXPECT_FLOAT_EQ(26.094543, battery_state_->temperature);
  EXPECT_FLOAT_EQ(0.18720642, battery_state_->percentage);
  EXPECT_FLOAT_EQ(3.7441282, battery_state_->charge);

  // For single battery if readings stay the same values of battery_1 and battery should be the same
  EXPECT_FLOAT_EQ(battery_1_state_->voltage, battery_state_->voltage);
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_state_->temperature);
  EXPECT_FLOAT_EQ(battery_1_state_->percentage, battery_state_->percentage);
  EXPECT_FLOAT_EQ(battery_1_state_->charge, battery_state_->charge);
}

TEST_F(TestBatteryNodeADCSingle, BatteryTimeout)
{
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // Battery state msg should have some values
  EXPECT_FALSE(std::isnan(battery_state_->voltage));
  EXPECT_FALSE(std::isnan(battery_state_->temperature));
  EXPECT_FALSE(std::isnan(battery_state_->current));
  EXPECT_FALSE(std::isnan(battery_state_->charge));
  EXPECT_FALSE(std::isnan(battery_state_->percentage));

  // Force error and wait for timeout
  std::filesystem::remove(std::filesystem::path(device0_path_ / "in_voltage2_raw"));
  std::filesystem::remove(std::filesystem::path(device1_path_ / "in_voltage2_raw"));
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(1000)));

  // Battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);
}

TEST_F(TestBatteryNodeADCSingle, BatteryCharging)
{
  // Wait for node to initialize
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // Publish charger connected state
  IOStateMsg io_state;
  io_state.charger_connected = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(1000)));

  EXPECT_NE(battery_state_->power_supply_status, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
}

TEST_F(TestBatteryNodeADCSingle, RoboteqInitOnADCFail)
{
  // Remove ADC device
  std::filesystem::remove_all(device0_path_);

  // Wait for node to initialize
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(5000)));

  // Battery state status should be UNKNOWN
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);

  // Publish driver state that should update the battery message
  RobotDriverStateMsg driver_state;
  driver_state.header.stamp = battery_driver_node_->get_clock()->now();
  driver_state_pub_->publish(driver_state);

  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    battery_driver_node_, battery_state_, std::chrono::milliseconds(1000)));

  // Battery state status should be different than UNKNOWN
  EXPECT_NE(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
