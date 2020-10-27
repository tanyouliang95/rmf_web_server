# Simple RMF Web client

This is a test repo to demonstrate the usage of dispatcher lib in `rmf_core`.
In short, this is to populate all rmf tasks and fleet information to a db.
And this will also define clear REST Interface to submit a task.

**Minimal Example: Work in Progress**

## Installation

Installation of `mongocxx`: 
 - [here](http://mongocxx.org/mongocxx-v3/installation/)
 - and [here](https://www.cnblogs.com/pluse/p/5491300.html)

Setup a MongoDB

## Run Nodes

```bash
ros2 run rmf_task_ros2 dispatcher_db_client
```

**Docker**
To download the the image, either get it from docker hub or the local 
github registry ("packages" on the right panel).

```bash
# Srv Dispatcher 
docker run -it tanyouliang95/rmf_core:web-server \
ros2 run rmf_task_ros2 rmf_task_dispatcher

# Srv Dispatcher with DB connnection
docker run -it --network host tanyouliang95/rmf_core:web-server \
ros2 run rmf_web_server dispatcher_db_client
```
