#include "any_node/any_node.hpp"

#include "any_node_example/ExampleNode.hpp"

int main(int argc, char** argv) {
  any_node::Nodewrap<any_node_example::ExampleNode> node(argc, argv, "exampleNode", 2);  // use 2 spinner threads
  return static_cast<int>(!node.execute());  // execute blocks until the node was requested to shut down (after reception of a signal (e.g.
                                             // SIGINT) or after calling the any_node::Node::shutdown() function)
}
