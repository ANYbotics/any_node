# Any worker

##Overview
This package provides a Worker and WorkerManager classes. Each Worker owns a thread, which calls a given callback function at given rate. Workers can be added/started and stopped any time. 
The user is responsible that the callback functions do not block (otherwise, the worker may not be able to terminate, even if requested to do so).
