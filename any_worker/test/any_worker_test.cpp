// gtest
#include <gtest/gtest.h>

// std
#include <sched.h>   // For sched_setscheduler
#include <unistd.h>  // For getpid()

#ifndef ROS2_BUILD
#include <message_logger/message_logger.hpp>
#else
#include <rclcpp/logging.hpp>
#endif

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  // Increase the priority of this process, as some of the tests are timing critical.
  sched_param params{};
  params.sched_priority = 20;
  if (sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0) {
#ifndef ROS2_BUILD
    MELO_WARN("Failed to set rtprio for the test - execution can be unreliable!");
#else
    RCLCPP_WARN(rclcpp::get_logger("any_worker"), "Failed to set rtprio for the test - execution can be unreliable!");
#endif
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
