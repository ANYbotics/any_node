// std
#include <cmath>
#include <thread>

// gtest
#include <gtest/gtest.h>

// any worker
#include "any_worker/Rate.hpp"


using namespace any_worker;


/*!
 * Simulate some processing which takes a certain amount of time.
 * @param duration Processing duration in seconds.
 */
void doSomething(const double duration) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e9 * duration)));
}


TEST(RateTest, Initialization)
{
    const double timeStep = 0.1;
    any_worker::Rate rate("Test", timeStep);

    EXPECT_EQ(rate.getTimeStep(), timeStep);
    EXPECT_EQ(rate.getMaxTimeStepWarning(), timeStep);
    EXPECT_EQ(rate.getMaxTimeStepError(), 10.0*timeStep);
    EXPECT_EQ(rate.getEnforceRate(), true);
    EXPECT_EQ(rate.getClockId(), CLOCK_MONOTONIC);
    EXPECT_EQ(rate.getNumTimeSteps(), 0);
    EXPECT_EQ(rate.getNumWarnings(), 0);
    EXPECT_EQ(rate.getNumErrors(), 0);
    EXPECT_TRUE(std::isnan(rate.getAwakeTime()));
    EXPECT_TRUE(std::isnan(rate.getAwakeTimeMean()));
    EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));
}

TEST(RateTest, Reset)
{
    const double timeStep = 0.1;
    const double processingTime = 0.05;

    // Run for one time step and reset.
    any_worker::Rate rate("Test", timeStep);
    doSomething(processingTime);
    rate.sleep();
    rate.reset();

    EXPECT_EQ(rate.getNumTimeSteps(), 0);
    EXPECT_EQ(rate.getNumWarnings(), 0);
    EXPECT_EQ(rate.getNumErrors(), 0);
    EXPECT_TRUE(std::isnan(rate.getAwakeTime()));
    EXPECT_TRUE(std::isnan(rate.getAwakeTimeMean()));
    EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));
}

TEST(RateTest, SleepWithEnforceRate)
{
    const double timeStep = 0.1;
    any_worker::Rate rate("Test", timeStep);
    rate.setEnforceRate(true);

    timespec start;
    timespec end;
    const double processingTime = 0.05;
    std::vector<double> processingTimes;
    std::vector<double> summedStepTimes;

    // Test sleep() without processing.
    clock_gettime(CLOCK_MONOTONIC, &start);
    rate.reset();
    rate.sleep();
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), timeStep, 0.001);

    // Test sleep() with processing additionally.
    clock_gettime(CLOCK_MONOTONIC, &start);
    rate.reset();
    doSomething(processingTime);
    rate.sleep();
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), timeStep, 0.001);

    // Test sleep() with where one step takes too long, recovery within one step.
    processingTimes = {
        0.02, 0.02, 0.15, 0.02, 0.02
    };
    summedStepTimes = {
        0.0, 0.1, 0.2, 0.35, 0.4, 0.5
    };
    rate.reset();
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[i], 0.002);
        doSomething(processingTimes[i]);
        rate.sleep();
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()], 0.002);

    // Test sleep() with where one step takes too long, recovery within two steps.
    processingTimes = {
        0.02, 0.02, 0.19, 0.02, 0.02, 0.02
    };
    summedStepTimes = {
        0.0, 0.1, 0.2, 0.39, 0.41, 0.5, 0.6
    };
    rate.reset();
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[i], 0.002);
        doSomething(processingTimes[i]);
        rate.sleep();
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()], 0.002);

    // Test sleep() with where two steps take too long, recovery within one step.
    processingTimes = {
        0.02, 0.02, 0.12, 0.12, 0.02, 0.02
    };
    summedStepTimes = {
        0.0, 0.1, 0.2, 0.32, 0.44, 0.5, 0.6
    };
    rate.reset();
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[i], 0.002);
        doSomething(processingTimes[i]);
        rate.sleep();
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()], 0.002);

    // Test sleep() with where two steps take too long, recovery within two steps.
    processingTimes = {
        0.02, 0.02, 0.12, 0.12, 0.08, 0.02, 0.02
    };
    summedStepTimes = {
        0.0, 0.1, 0.2, 0.32, 0.44, 0.52, 0.6, 0.7
    };
    rate.reset();
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[i], 0.002);
        doSomething(processingTimes[i]);
        rate.sleep();
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()], 0.002);
}

TEST(RateTest, SleepWithoutEnforceRate)
{
    const double timeStep = 0.1;
    any_worker::Rate rate("Test", timeStep);
    rate.setEnforceRate(false);

    timespec start;
    timespec end;
    const double processingTime = 0.05;
    std::vector<double> processingTimes;
    std::vector<double> summedStepTimes;

    // Test sleep() without processing.
    clock_gettime(CLOCK_MONOTONIC, &start);
    rate.reset();
    rate.sleep();
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), timeStep, 0.001);

    // Test sleep() with processing.
    clock_gettime(CLOCK_MONOTONIC, &start);
    rate.reset();
    doSomething(processingTime);
    rate.sleep();
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), timeStep, 0.001);

    // Test sleep() with where one step takes too long.
    processingTimes = {
        0.02, 0.02, 0.15, 0.02, 0.02
    };
    summedStepTimes = {
        0.0, 0.1, 0.2, 0.35, 0.45, 0.55
    };
    rate.reset();
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[i], 0.002);
        doSomething(processingTimes[i]);
        rate.sleep();
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()], 0.002);

    // Test sleep() with where two steps take too long.
    processingTimes = {
        0.02, 0.02, 0.12, 0.12, 0.02, 0.02
    };
    summedStepTimes = {
        0.0, 0.1, 0.2, 0.32, 0.44, 0.54, 0.64
    };
    rate.reset();
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[i], 0.004);
        doSomething(processingTimes[i]);
        rate.sleep();
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(Rate::GetDuration(start, end), summedStepTimes[processingTimes.size()], 0.004);
}

TEST(RateTest, WarningsAndErrors)
{
    const double timeStep = 0.1;
    any_worker::Rate rate("Test", timeStep);
    doSomething(0.5*timeStep); // Ok
    rate.sleep();
    doSomething(2.0*timeStep); // Warning
    rate.sleep();
    doSomething(3.0*timeStep); // Warning
    rate.sleep();
    doSomething(11.0*timeStep); // Error
    rate.sleep();

    EXPECT_EQ(rate.getNumTimeSteps(), 4);
    EXPECT_EQ(rate.getNumWarnings(), 2);
    EXPECT_EQ(rate.getNumErrors(), 1);
}

TEST(RateTest, StatisticsWithEnforceRate)
{
    const double timeStep = 0.1;
    any_worker::Rate rate("Test", timeStep);
    rate.setEnforceRate(true);

    // Test 1 time step.
    const double processingTime = 0.05;
    rate.reset();
    doSomething(processingTime);
    rate.sleep();

    EXPECT_EQ(rate.getNumTimeSteps(), 1);
    EXPECT_NEAR(rate.getAwakeTime(), processingTime, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), processingTime, 0.001);
    EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));

    // Test 10 time steps with similar processing times.
    const unsigned int numTimeSteps = 10;
    rate.reset();
    for (unsigned int i = 0; i < numTimeSteps; i++) {
        doSomething(processingTime);
        rate.sleep();
    }

    EXPECT_EQ(rate.getNumTimeSteps(), numTimeSteps);
    EXPECT_NEAR(rate.getAwakeTime(), processingTime, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), processingTime, 0.001);
    EXPECT_LE(rate.getAwakeTimeStdDev(), 0.001);
    EXPECT_FALSE(rate.getAwakeTimeStdDev() == 0.0); // If it is 0.0 something is fishy.

    // Test 10 time steps with different processing times.
    // Example values from https://en.wikipedia.org/wiki/Standard_deviation
    const std::vector<double> processingTimes = {
        0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09
    };
    rate.reset();
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        doSomething(processingTimes[i]);
        rate.sleep();
    }

    EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
    EXPECT_NEAR(rate.getAwakeTime(), 0.09, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, 0.003);

    // Test again with time step violation.
    rate.setTimeStep(0.035);
    rate.reset();
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        doSomething(processingTimes[i]);
        rate.sleep();
    }

    EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
    EXPECT_NEAR(rate.getAwakeTime(), 0.09, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, 0.003);
}

TEST(RateTest, StatisticsWithoutEnforceRate)
{
    const double timeStep = 0.1;
    any_worker::Rate rate("Test", timeStep);
    rate.setEnforceRate(false);

    // Test 1 time step.
    const double processingTime = 0.05;
    rate.reset();
    doSomething(processingTime);
    rate.sleep();

    EXPECT_EQ(rate.getNumTimeSteps(), 1);
    EXPECT_NEAR(rate.getAwakeTime(), processingTime, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), processingTime, 0.001);
    EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));

    // Test 10 time steps with similar processing times.
    const unsigned int numTimeSteps = 10;
    rate.reset();
    for (unsigned int i = 0; i < numTimeSteps; i++) {
        doSomething(processingTime);
        rate.sleep();
    }

    EXPECT_EQ(rate.getNumTimeSteps(), numTimeSteps);
    EXPECT_NEAR(rate.getAwakeTime(), processingTime, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), processingTime, 0.001);
    EXPECT_LE(rate.getAwakeTimeStdDev(), 0.001);
    EXPECT_FALSE(rate.getAwakeTimeStdDev() == 0.0); // If it is 0.0 something is fishy.

    // Test 10 time steps with different processing times.
    // Example values from https://en.wikipedia.org/wiki/Standard_deviation
    const std::vector<double> processingTimes = {
        0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09
    };
    rate.reset();
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        doSomething(processingTimes[i]);
        rate.sleep();
    }

    EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
    EXPECT_NEAR(rate.getAwakeTime(), 0.09, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, 0.003);

    // Test again with time step violation.
    rate.setTimeStep(0.035);
    rate.reset();
    for (unsigned int i = 0; i < processingTimes.size(); i++) {
        doSomething(processingTimes[i]);
        rate.sleep();
    }

    EXPECT_EQ(rate.getNumTimeSteps(), processingTimes.size());
    EXPECT_NEAR(rate.getAwakeTime(), 0.09, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, 0.001);
    EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, 0.003);
}

