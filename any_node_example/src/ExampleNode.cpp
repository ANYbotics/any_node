#include "any_node_example/ExampleNode.hpp"

namespace any_node_example {

bool ExampleNode::init() {
  // called on startup
  // add new workers (threads) like this. The workers are automatically stopped before the cleanup() function is called
  constexpr double defaultWorkerTimeStep = 3.0;
  constexpr int priority = 10;
  auto workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);
  addWorker("exampleNode::updateWorker", workerTimeStep, &ExampleNode::update, this, priority);

  // to create publishers/subscribers/service clients/servers or read params, use the functions provided by the any_node::Node base class
  // the name of the publishers/subscribers/.. has to be consistent
  // with the names specified in the yaml file (see description of Topic.hpp above)
  constexpr unsigned int defaultQueueSize = 1;
  ros::Publisher my_publisher = advertise<std_msgs::String>("my_publisher_name", "/default_publisher_topic", defaultQueueSize);
  ros::Subscriber my_subscriber =
      subscribe("my_subscriber_name", "/default_subscriber_topic", defaultQueueSize, &ExampleNode::subscriberCallback, this);

  MELO_INFO("init called");

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

void ExampleNode::cleanup() {
  // this function is called when the node is requested to shut down, _after_ the ros spinners and workers were stopped
  // no need to stop workers which are started with addWorker(..) function
  MELO_INFO("cleanup called");
}

bool ExampleNode::update(const any_worker::WorkerEvent& event) {
  // called by the worker which is automatically set up if rosparam standalone == True.
  // The frequency is defined in the time_step rosparam.
  MELO_INFO("update called");
  return true;
}

void ExampleNode::preCleanup() {
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void ExampleNode::subscriberCallback(const std_msgs::StringConstPtr& msg) {
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}

}  // namespace any_node_example
