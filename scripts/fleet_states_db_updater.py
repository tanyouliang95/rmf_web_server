
# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import rclpy
import math
from rclpy.node import Node
from rclpy.time import Time
from rmf_fleet_msgs.msg import FleetState

import pymongo

################################################################################

myclient = pymongo.MongoClient("mongodb://localhost:27017/")
mydb = myclient["rmf_task"]
mycol = mydb["fleet_states"]


class FleetStateListener(Node):
    def __init__(self):
        super().__init__('fleet_state_db_updater')
        print(f"Fleet state db updater is alive!")

        self.fleet_state_subscription = self.create_subscription(
            FleetState, 'fleet_states', self.fleet_state_cb, 1)

        # update freq every 3s
        self.timer = self.create_timer(3.0, self.timer_callback)

        self.fleet_states_dict = {}

    def fleet_state_cb(self, msg: FleetState):
        fleet_name = msg.name
        self.fleet_states_dict[fleet_name] = msg.robots

    def timer_callback(self):
        print(f" Update DB with: {len(self.fleet_states_dict)} fleets")

        for fleet_name, robot_states in self.fleet_states_dict.items():
            query = {"fleet_name": fleet_name}
            robots = self.convert_msg(robot_states)

            if (mycol.find_one(query) == None):
                mydict = {"fleet_name": fleet_name, "robots": robots}
                mycol.insert_one(mydict)
            else:
                values = {"$set": {"robots": robots}}
                mycol.update_one(query, values)

    def convert_msg(self, robot_states):
        
        bots = []
        for bot in robot_states:
            state = {}
            state["name"] = bot.name
            state["mode"] = bot.mode.mode
            state["battery_percent:"] = bot.battery_percent
            # time is missing here
            state["location_x:"] = bot.location.x
            state["location_y:"] = bot.location.y
            state["location_yaw:"] = bot.location.yaw
            state["level_name:"] = bot.location.yaw
            bots.append(state)

        return bots

################################################################################

def main(args=None):
    rclpy.init(args=args)
    fleet_state_listener = FleetStateListener()
    rclpy.spin(fleet_state_listener)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
