# Simple RMF Web client

Essentially this is to populate all rmf tasks and fleet information to a db.
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

Docker
```bash
# Srv Dispatcher
docker run -it osrf/rmf_core:web-server \
ros2 run rmf_task_ros2 rmf_task_dispatcher

# DB Access
docker run -it --network host osrf/rmf_core:web-server \
ros2 run rmf_web_server dispatcher_db_client
```
