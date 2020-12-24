/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <rclcpp/rclcpp.hpp>
#include <rmf_task_ros2/Dispatcher.hpp>

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task_list.hpp>

#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/exception/operation_exception.hpp>

//==============================================================================
using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
using GetTaskListSrv = rmf_task_msgs::srv::GetTaskList;

//==============================================================================
using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::open_document;

//==============================================================================
class MongoStatusClient
{
public:

  MongoStatusClient()
  {
    mongocxx::instance inst{};
    conn = mongocxx::uri{};

    auto collection = conn["rmf_task"]["dispatch_task"];
    auto cursor = collection.find({});
  }

  // insert to mongodb
  bool insert_status(const rmf_task_ros2::TaskStatus& status)
  {
    std::cout << " [DB INSERT] :: "
              << status.task_profile.task_id << std::endl;

    bsoncxx::builder::stream::document document{};
    document << "_id" << status.task_profile.task_id;
    document << "task_type" 
             << (int)status.task_profile.description.task_type.type;
    document << "submission_time"  // time conversion! todo
             << bsoncxx::types::b_date{std::chrono::system_clock::now()};

    // document << "params" << open_document;
    // for (auto param : status.task_profile.params)
    //   document << param.first << param.second;
    // document << close_document;

    document << "fleet_name" << status.fleet_name;
    document << "robot_name" << status.robot_name;
    document << "state" << (int)status.state;
    document << "status" << status.status;
    document << "is_done" << status.is_terminated();

    try
    {
      auto collection = conn["rmf_task"]["dispatch_task"];
      collection.insert_one(document.view());
    }
    catch (const mongocxx::operation_exception& e)
    {
      std::cerr<< e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool update_status(const rmf_task_ros2::TaskStatus& status)
  {
    std::cout << " [DB UPDATE] :: "
              << status.task_profile.task_id << std::endl;

    bsoncxx::builder::stream::document document{};
    document << "$set" << open_document;
    document << "fleet_name" << status.fleet_name;
    document << "robot_name" << status.robot_name;
    document << "state" << (int)status.state;
    document << "status" << status.status;
    document << "is_done" << status.is_terminated();
    document << close_document;

    bsoncxx::builder::stream::document header{};
    header << "_id" << status.task_profile.task_id;

    try
    {
      auto collection = conn["rmf_task"]["dispatch_task"];
      collection.update_one(header.view(), document.view());
    }
    catch (const mongocxx::operation_exception& e)
    {
      std::cerr<< e.what() << std::endl;
      return false;
    }
    return true;
  }

private:
  mongocxx::client conn;
};

//==============================================================================
int main(int argc, char* argv[])
{
  std::cout << "~Initializing Dispatcher DB Node~" << std::endl;
  rclcpp::init(argc, argv);

  auto dispatcher = rmf_task_ros2::Dispatcher::make_node(
    "rmf_dispatcher_node");

  const auto& node = dispatcher->node();
  RCLCPP_INFO(node->get_logger(), "Starting task dispatcher node");

  // start mongo db client instance
  MongoStatusClient task_db_conn;

  dispatcher->on_change(
    [&task_db_conn](const rmf_task_ros2::TaskStatusPtr status)
    {
      // crude way to update DB
      if (status->state == rmf_task_ros2::TaskStatus::State::Pending)
        task_db_conn.insert_status(*status);
      else
        task_db_conn.update_status(*status);
    }
  );

  dispatcher->spin();

  RCLCPP_INFO(node->get_logger(), "Closing down task dispatcher node");
}
