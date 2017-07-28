# signal_handler (a.k.a. minimal nodewrap)

##Overview
Implements several convenience classes and functions.

### Param.hpp
Forwards to param_io package, which allows to read and write ROS messages from and to the parameter server. Also, it provides param(..) functions which print a warning if a requested parameter was not found.

### Topic.hpp
Allows to advertise/subscribe to/from topics/services, whose connection details (topic name, latched, queue_size, deactivate, ...) are saved as ros parameters. Example yaml file:

    publishers:
      my_publisher_name:
        topic: /my_publisher_topic_name
        queue_size: 1
        latch: false

    subscribers:
      my_subscriber_name:
        topic: /my_subscriber_topic_name
        queue_size: 1

    servers:
      my_service_server_name:
        service: my_service_name

    clients:
      my_service_client_name:
        service: my_service_name
        persistent: false


### Node.hpp
Provides an interface base class signal_handler::Node, which declares init, cleanup and update functions and has a any_worker::WorkerManager instance.
Classes derived from this are compatible with the Nodewrap and rtcontrol templates.
Additionally, it forwards calls of subscribe, advertise, param, advertiseService serviceClient and addWorker calls to the above mentioned functions.

### Nodewrap.hpp
Convencience template, designed to be used with classes derived from signal_handler::Node. It automatically sets up ros nodehandlers (with private namespace) and spinners, signal handlers (like SIGINT, ...) and calls the init, cleanup and (optionally) update functions of the given Node. The following ros parameters are used:

- standalone: If set to true, a worker will be set up that calls the Node::update(..) function with a given freqeuncy (see time_step) 
- time_step: Time step between consecutive calls of the Node::update(..) function. Set to 0 to run only once. Ignored if standalone is set to false.

Example:

    class ExampleNode : public signal_handler::Node {
    public:
        ExampleNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
        ExampleNode(signal_handler::Node::NodeHandlePtr nh): signal_handler::Node(nh)
        {
        }
        
        virtual ~ExampleNode()
        {
        }
        
        void init() // called on startup
        {
            // add new workers (threads) like this. The workers are automatically stopped before the cleanup() function is called
            const double worker_timestep = 0.01;
            addWorker("my_worker_name", worker_timestep, &ExampleNode::myWorkerFunction, this);

            // to create publishers/subscribers/service clients/servers or read params, use the functions provided by the signal_handler::Node base class
            // the name of the publishers/subscribers/.. has to be consistent 
            // with the names specified in the yaml file (see description of Topic.hpp above)
            const unsigned int defaultQueueSize = 1;
            ros::Publisher my_publisher = advertise<my_msg>("my_publisher_name", "/default_topic", defaultQueueSize);

            ros::Subscriber my_subscriber = subscribe("my_subscriber_name", "/default_topic", defaultQueueSize, &ExampleNode::subscriberCallback, this);

            const double defaultParamValue = 5.75;
            const double myParam = param<double>("myParamName", defaultParamValue);

        }
        
        void cleanup() // called on shutdown
        {
        }
        
        bool update(const any_worker::WorkerEvent& event) // called by the worker which is automatically set up if rosparam standalone == True.
                                                          // The frequency is defined in the time_step rosparam.
        {
            return true;
        }

        bool myWorkerFunction(const any_worker::WorkerEvent& event) // called by the worker created in init() function
        {

        }

        void subscriberCallback(const subscriber_msgConstPtr &msg)
        {

        }
        
    };
    
    int main(int argc, char **argv) {
        signal_handler::Nodewrap<ExampleNode> node(argc, argv, "exampleNode", 4); // use 4 spinner threads
        node.execute(90); // 90=priority of the thread calling the update(..) function (if any)
        // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the signal_handler::Node::shutdown() function) 
        return 0;
    }

