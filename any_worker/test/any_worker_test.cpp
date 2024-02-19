// gtest
#include <gtest/gtest.h>

// std
#include <sched.h>   // For sched_setscheduler
#include <unistd.h>  // For getpid()

#include <message_logger/message_logger.hpp>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  // Increase the priority of this process, as some of the tests are timing critical.
  sched_param params{};
  params.sched_priority = 20;
  if (sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0) {
    MELO_WARN("Failed to set rtprio for the test - execution can be unreliable!");
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
