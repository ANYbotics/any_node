/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Remo Diethelm
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file    Rate.hpp
 * @author  Remo Diethelm
 * @date    January, 2018
 */

#pragma once


// std
#include <atomic>
#include <ctime>
#include <string>


namespace any_worker {


/*!
 * Rate class for repeated processing at a certain rate.
 * The processing time is defined as the "awake" time, the time between the "sleep"
 * time. This class implements a sleep() method for sleeping for the remaining time
 * of the time step. Also statistics about the awake time are taken.
 *
 * If the awake time is bigger than the time step (timing is violated),
 * there are two behaviors implemented:
 *
 * Enforce the rate: Try to keep the rate constant by compensating with shorter time steps.
 * |       |       |       |       |       |       |
 * |===--->|===--->|============|===|===-->|===--->|
 * |       |       |       |       |       |       |
 *
 * Do not enforce the rate: Try to keep the single time step constant.
 * |       |       |            |       |       |       |
 * |===--->|===--->|============|===--->|===--->|===--->|
 * |       |       |            |       |       |       |
 *
 * Legend:
 *  |  Step time
 * === Awake time
 * --> Sleep time
 *
 * The time step, the time step thresholds for warnings and errors and the setting to
 * enforce the rate can be changed during run-time of this class.
 */
class Rate
{
protected:
    //! Factor storing nanoseconds per seconds.
    static constexpr long int NSecPerSec_ = 1e9;
    //! Factor storing seconds per nanoseconds.
    static constexpr double SecPerNSec_ = 1.0/NSecPerSec_;

    //! Time step in seconds.
    std::atomic<double> timeStep_;
    //! Max time step for warnings in seconds.
    std::atomic<double> maxTimeStepWarning_;
    //! Max time step for errors in seconds.
    std::atomic<double> maxTimeStepError_;
    //! Boolean indicating whether the rate should be enforced.
    std::atomic<bool> enforceRate_;
    //! Linux clock ID.
    const clockid_t clockId_ = 0;

    //! Name for printing.
    std::string name_;
    //! Point in time when the most recent sleep() started.
    timespec sleepStartTime_;
    //! Point in time when the most recent sleep() ended.
    timespec sleepEndTime_;
    //! Point in time when the most recent step should have started.
    //! If the timing is fine, the step time is equal to the sleep end time.
    timespec stepTime_;
    //! Counter storing how many times sleep has been called.
    unsigned int numTimeSteps_ = 0;
    //! Counter storing how many times a time step took longer than the warning threshold.
    unsigned int numWarnings_ = 0;
    //! Counter storing how many times a time step took longer than the error threshold, not considering warnings.
    unsigned int numErrors_ = 0;
    //! Most recent time which elapsed between subsequent calls of sleep().
    double awakeTime_ = 0.0;
    //! Mean of the time which elapsed between subsequent calls of sleep().
    double awakeTimeMean_ = 0.0;
    //! Helper variable to compute the variance of the time step which elapsed between subsequent calls of sleep().
    double awakeTimeM2_ = 0.0;

public:
    /*!
     * Simple constructor.
     * Starts the clock. Call reset() to restart it if you do not intend to call sleep() immediately.
     * @param name     Name for printing.
     * @param timeStep Time step in seconds.
     */
    Rate(const std::string& name,
         const double timeStep);

    /*!
     * Advanced constructor.
     * Starts the clock. Call reset() to restart it if you do not intend to call sleep() immediately.
     * @param name             Name for printing.
     * @param timeStep         Time step in seconds.
     * @param maxTimeStepWarn  Max time step for warnings in seconds.
     * @param maxTimeStepError Max time step for errors in seconds.
     * @param enforceRate      Enforce the rate.
     * @param clockId          Linux clock ID.
     */
    Rate(const std::string& name,
         const double timeStep,
         const double maxTimeStepWarn,
         const double maxTimeStepError,
         const bool enforceRate,
         const clockid_t clockId);

    /*!
     * Move operator for atomics.
     * @param other Rate to move.
     */
    Rate(Rate&& other);

    /*!
     * Get the name.
     * @return Name.
     */
    const std::string& getName() const;

    /*!
     * Get the time step in seconds.
     * @return Time step in seconds.
     */
    double getTimeStep() const;

    /*!
     * Set the time step in seconds.
     * @param timeStep Time step in seconds.
     */
    void setTimeStep(const double timeStep);

    /*!
     * Get the max time step for warnings in seconds.
     * @return Max time step for warnings in seconds.
     */
    double getMaxTimeStepWarning() const;

    /*!
     * Set the max time step for warnings in seconds.
     * @param maxTimeStepWarn Max time step for warnings in seconds.
     */
    void setMaxTimeStepWarning(const double maxTimeStepWarn);

    /*!
     * Get the max time step for errors in seconds.
     * @return Max time step for errors in seconds.
     */
    double getMaxTimeStepError() const;

    /*!
     * Set the max time step for errors in seconds.
     * @param maxTimeStepError Max time step for errors in seconds.
     */
    void setMaxTimeStepError(const double maxTimeStepError);

    /*!
     * Get enforce rate.
     * @return Enforce rate.
     */
    bool getEnforceRate() const;

    /*!
     * Enable or disable enforcing the rate.
     * @param enforceRate Enforce rate.
     */
    void setEnforceRate(const bool enforceRate);

    /*!
     * Get the Linux clock ID.
     * @return Linux clock ID.
     */
    clockid_t getClockId() const;

    /*!
     * Reset the internal memory and restart the sleep time.
     */
    void reset();

    /*!
     * Sleep for the rest of the time step.
     */
    void sleep();

    /*!
     * Get the time when the most recent sleep() started.
     * @return Time when the most recent sleep() started.
     */
    const timespec& getSleepStartTime() const;

    /*!
     * Get the time when the most recent sleep() was supposed to end.
     * @return Time when the most recent sleep() was supposed to end.
     */
    const timespec& getSleepEndTime() const;

    /*!
     * Get the time when the most recent step should have started.
     * @return Time when the most recent step should have started.
     */
    const timespec& getStepTime() const;

    /*!
     * Get the number of time steps.
     * @return Number of time steps.
     */
    unsigned int getNumTimeSteps() const;

    /*!
     * Get the number of warnings.
     * @return Number of warnings.
     */
    unsigned int getNumWarnings() const;

    /*!
     * Get the number of errors, not considering warnings.
     * @return Number of errors, not considering warnings.
     */
    unsigned int getNumErrors() const;

    /*!
     * Get the most recent time which elapsed between subsequent calls of sleep().
     * @return Elapsed time in seconds.
     */
    double getAwakeTime() const;

    /*!
     * Get the mean of the time which elapsed between subsequent calls of sleep().
     * @return Mean in seconds.
     */
    double getAwakeTimeMean() const;

    /*!
     * Get the variance of the time which elapsed between subsequent calls of sleep().
     * @return Variance in seconds^2.
     */
    double getAwakeTimeVar() const;

    /*!
     * Get the standard deviation of the time which elapsed between subsequent calls of sleep().
     * @return Standard deviation in seconds.
     */
    double getAwakeTimeStdDev() const;

    /*!
     * Get the duration between start and end time points.
     * @param start Start time point.
     * @param end   End time point.
     * @return Duration in seconds.
     */
    static double GetDuration(const timespec& start, const timespec& end);

protected:
    /*!
     * Check if a time step in seconds is valid.
     * @param timeStep Time step in seconds.
     * @return True if valid.
     */
    static bool TimeStepIsValid(const double timeStep);

    /*!
     * Check if a max elapsed time in seconds is valid.
     * @param maxTimeStep Max time step in seconds.
     * @return True if valid.
     */
    static bool MaxTimeStepIsValid(const double maxTimeStep);
};


} // namespace any_worker
