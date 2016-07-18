# Any_node (a.k.a. minimal nodewrap)

##Overview
Implements several convenience classes and functions.

### Param.hpp
Forwards to param_io package, which allows to read and write ROS messages from and to the parameter server. Also, it provides param(..) functions which print a warning if a requested parameter was not found.

### Topic.hpp
Allows to advertise/subscribe to/from topics/services, whose connection details (topic name, latched, queue_size, deactivate, ...) are saved as ros parameters.

### Node.hpp
Provides an interface base class any_node::Node, which declares init, cleanup and update functions and has a any_worker::WorkerManager instance.
Classes derived from this are compatible with the Nodewrap and rtcontrol templates.
Additionally, it forwards calls of subscribe, advertise, param, advertiseService serviceClient and addWorker calls to the above mentioned functions.

### Nodewrap.hpp
Convencience template, designed to be used with classes derived from any_node::Node. It automatically sets up ros nodehandlers (with private namespace) and spinners, signal handlers (like SIGINT, ...) and calls the init, cleanup and (optionally) update functions of the given Node. The following ros parameters are used:

- standalone: If set to true, a worker will be set up that calls the Node::update(..) function with a given freqeuncy (see time_step) 
- time_step: Time step between consecutive calls of the Node::update(..) function. Set to 0 to run only once. Ignored if standalone is set to false.

Example:

    class exampleNode : public any_node::Node {
    public:
        exampleNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
        exampleNode(NodeHandlePtr nh): any_node::Node(nh)
        {
        }
        
        virtual ~exampleNode()
        {
        }
        
        void init() // called on startup
        {
        }
        
        void cleanup() // called on shutdown
        {
        }
        
        bool update(const any_worker::WorkerEvent& event) // called on every update step of the node. The frequency is defined in the time_step rosparam.
        {
            return true;
        }
        
    };
    
    int main(int argc, char **argv) {
        any_node::Nodewrap<exampleNode> node(argc, argv, "exampleNode", 4); // use 4 spinner threads
        node.execute(90); // 90=priority of the thread calling the update(..) function (if any)
        // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function) 
        return 0;
    }

