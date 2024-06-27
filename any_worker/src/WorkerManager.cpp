/*!
 * @file	WorkerManager.cpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#include "any_worker/WorkerManager.hpp"
#ifndef ROS2_BUILD
#include "message_logger/message_logger.hpp"
#else
#include <rclcpp/logging.hpp>
#endif

namespace any_worker {

WorkerManager::WorkerManager() : workers_(), mutexWorkers_() {}

WorkerManager::~WorkerManager() {
  cancelWorkers();
}

bool WorkerManager::addWorker(const WorkerOptions& options, const bool autostart) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto insertedElement = workers_.emplace(options.name_, Worker(options));
  if (!insertedElement.second) {
#ifndef ROS2_BUILD
    MELO_ERROR("Failed to create worker [%s]", options.name_.c_str());
#else
    RCLCPP_ERROR(rclcpp::get_logger("any_worker"), "Failed to create worker [%s]", options.name_.c_str());
#endif
    return false;
  }
  if (autostart) {
    return insertedElement.first->second.start();
  }
  return true;
}

// bool WorkerManager::addWorker(Worker&& worker) {
//    std::lock_guard<std::mutex> lock(mutexWorkers_);
//    auto insertedElement = workers_.emplace( worker.getName(), std::move(worker) );
//    if(!insertedElement.second) {
//        MELO_ERROR("Failed to move worker [%s]", worker.getName().c_str());
//        return false;
//    }
//    return true;
//}

void WorkerManager::startWorker(const std::string& name, const int priority) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
#ifndef ROS2_BUILD
    MELO_ERROR("Cannot start worker [%s], worker not found", name.c_str());
#else
    RCLCPP_ERROR(rclcpp::get_logger("any_worker"), "Cannot start worker [%s], worker not found", name.c_str());
#endif
    return;
  }
  worker->second.start(priority);
}

void WorkerManager::startWorkers() {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto& worker : workers_) {
    worker.second.start();
  }
}

void WorkerManager::stopWorker(const std::string& name, const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
#ifndef ROS2_BUILD
    MELO_ERROR("Cannot stop worker [%s], worker not found", name.c_str());
#else
    RCLCPP_ERROR(rclcpp::get_logger("any_worker"), "Cannot stop worker [%s], worker not found", name.c_str());
#endif
    return;
  }
  worker->second.stop(wait);
}

void WorkerManager::stopWorkers(const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto& worker : workers_) {
    worker.second.stop(wait);
  }
}

bool WorkerManager::hasWorker(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  return (workers_.find(name) != workers_.end());
}

void WorkerManager::cancelWorker(const std::string& name, const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
#ifndef ROS2_BUILD
    MELO_ERROR("Cannot stop worker [%s], worker not found", name.c_str());
#else
    RCLCPP_ERROR(rclcpp::get_logger("any_worker"), "Cannot stop worker [%s], worker not found", name.c_str());
#endif
    return;
  }
  worker->second.stop(wait);
  workers_.erase(worker);
}

void WorkerManager::cancelWorkers(const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);

  // signal all workers to stop
  for (auto& worker : workers_) {
    worker.second.stop(wait);
  }

  // call destructors of all workers, which will join the underlying thread
  workers_.clear();
}

void WorkerManager::setWorkerTimestep(const std::string& name, const double timeStep) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
#ifndef ROS2_BUILD
    MELO_ERROR("Cannot change timestep of worker [%s], worker not found", name.c_str());
#else
    RCLCPP_ERROR(rclcpp::get_logger("any_worker"), "Cannot change timestep of worker [%s], worker not found", name.c_str());
#endif
    return;
  }
  worker->second.setTimestep(timeStep);
}

void WorkerManager::cleanDestructibleWorkers() {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto it = workers_.begin(); it != workers_.end();) {
    if (it->second.isDestructible()) {
      it = workers_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace any_worker
