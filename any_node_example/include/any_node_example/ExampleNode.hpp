#include "any_node/Node.hpp"

#include <std_msgs/String.h>

namespace any_node_example {
class ExampleNode : public any_node::Node {
public:
  ExampleNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  ExampleNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh)
  {
  }

  ~ExampleNode() override
  {
  }

  // these 3 functions need to be implemented
  void init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent& event) override;

  // this function implementation is optional (default is empty)
  void preCleanup() override;

  bool myWorkerFunction(const any_worker::WorkerEvent& event);
  void subscriberCallback(const std_msgs::StringConstPtr &msg);
};

}
