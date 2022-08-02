// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <list>
#include <memory>
#include <string>

// include ROS
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <ros/console.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include "bridge.hpp"

// Direction of bridge.
enum Direction
{
  // Both directions.
  BIDIRECTIONAL = 0,
  // Only from IGN to ROS
  FROM_IGN_TO_ROS = 1,
  // Only from ROS to IGN
  FROM_ROS_TO_IGN = 2,
};

//////////////////////////////////////////////////
void usage()
{
  ROS_INFO_STREAM(
      "Bridge a collection of ROS and Ignition Transport topics.\n\n"
      << "  parameter_bridge <topic@ROS_type(@,[,])Ign_type> .. "
      << " <topic@ROS_type(@,[,])Ign_type>\n\n"
      << "The first @ symbol delimits the topic name from the message types.\n"
      << "Following the first @ symbol is the ROS message type.\n"
      << "The ROS message type is followed by an @, [, or ] symbol where\n"
      << "    @  == a bidirectional bridge, \n"
      << "    [  == a bridge from Ignition to ROS,\n"
      << "    ]  == a bridge from ROS to Ignition.\n"
      << "Following the direction symbol is the Ignition Transport message "
      << "type.\n\n"
      << "A bidirectional bridge example:\n"
      << "    parameter_bridge /chatter@std_msgs/String@ignition.msgs"
      << ".StringMsg\n\n"
      << "A bridge from Ignition to ROS example:\n"
      << "    parameter_bridge /chatter@std_msgs/String[ignition.msgs"
      << ".StringMsg\n\n"
      << "A bridge from ROS to Ignition example:\n"
      << "    parameter_bridge /chatter@std_msgs/String]ignition.msgs"
      << ".StringMsg" << std::endl);
}

int parseArguments(int argc, char * argv[], 
  std::list<ros_ign_bridge::BridgeHandles>& bidirectional_handles, 
  std::list<ros_ign_bridge::BridgeIgnToRosHandles>& ign_to_ros_handles,
  std::list<ros_ign_bridge::BridgeRosToIgnHandles>& ros_to_ign_handles,
  std::shared_ptr<ignition::transport::Node> ign_node,
  ros::NodeHandle& ros_node,
  std::shared_ptr<std::map<std::string, std::string>> tf_to_ign, 
  std::shared_ptr<std::map<std::string, std::string>> ign_to_tf) {
// Parse all arguments.
  const std::string delim = "@";
  const std::string delim2 = "[";
  const std::string delim3 = "]";
  const size_t queue_size = 10;
  for (auto i = 1; i < argc; ++i)
  {
    std::string ros_topic;
    std::string ign_topic;
    bool different_topics = false;
    
    std::string arg = std::string(argv[i]);
  
    // this works only for char delimiters
    int count = std::count(arg.begin(), arg.end(), delim.c_str()[0]);
    count += std::count(arg.begin(), arg.end(), delim2.c_str()[0]);
    count += std::count(arg.begin(), arg.end(), delim3.c_str()[0]);

    if (count == 3) {
      different_topics = true;
    }

    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0)
    {
     usage();
     return -1;
    }
    ros_topic = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    if (different_topics) {
      delimPos = arg.find(delim);
      if (delimPos == std::string::npos || delimPos == 0)
      {
        usage();
        return -1;
      }
      ign_topic = arg.substr(0, delimPos);
      arg.erase(0, delimPos + delim.size());
    }
    else {
      ign_topic = ros_topic;
    }

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   [ == only from IGN to ROS, or
    //   ] == only from ROS to IGN.
    delimPos = arg.find("@");
    Direction direction = BIDIRECTIONAL;
    if (delimPos == std::string::npos || delimPos == 0)
    {
      delimPos = arg.find("[");
      if (delimPos == std::string::npos || delimPos == 0)
      {
        delimPos = arg.find("]");
        if (delimPos == std::string::npos || delimPos == 0)
        {
          usage();
          return -1;
        }
        else
        {
          direction = FROM_ROS_TO_IGN;
        }
      }
      else
      {
        direction = FROM_IGN_TO_ROS;
      }
    }
    std::string ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty())
    {
      usage();
      return -1;
    }
    std::string ign_type_name = arg;

    try
    {
      switch (direction)
      {
        default:
        case BIDIRECTIONAL:
          bidirectional_handles.push_back(
              ros_ign_bridge::create_bidirectional_bridge(
                ros_node, ign_node,
                ros_type_name, ign_type_name,
                ros_topic, ign_topic,
                tf_to_ign, ign_to_tf, 
                queue_size));
          break;
        case FROM_IGN_TO_ROS:
          ign_to_ros_handles.push_back(
              ros_ign_bridge::create_bridge_from_ign_to_ros(
                ign_node, ros_node,
                ign_type_name, ign_topic, queue_size,
                ros_type_name, ros_topic, queue_size,
                tf_to_ign, ign_to_tf));
          break;
        case FROM_ROS_TO_IGN:
          ros_to_ign_handles.push_back(
              ros_ign_bridge::create_bridge_from_ros_to_ign(
                ros_node, ign_node,
                ros_type_name, ros_topic, queue_size,
                ign_type_name, ign_topic, queue_size,
                tf_to_ign, ign_to_tf));
          break;
      }
    }
    catch (std::runtime_error &_e)
    {
      ROS_ERROR_STREAM("Failed to create a bridge for topics ["
          << ros_topic << " " << ign_topic << "] "
          << "with ROS type [" << ros_type_name << "] and "
          << "Ignition Transport type [" << ign_type_name << "]"
          << std::endl);
    }
  }
  return 0;
}

int parseParameters(bool topics_available, bool frame_available,
  ros::NodeHandle& nh_private,
  std::list<ros_ign_bridge::BridgeHandles>& bidirectional_handles, 
  std::list<ros_ign_bridge::BridgeIgnToRosHandles>& ign_to_ros_handles,
  std::list<ros_ign_bridge::BridgeRosToIgnHandles>& ros_to_ign_handles,
  std::shared_ptr<ignition::transport::Node> ign_node,
  ros::NodeHandle& ros_node,
  std::shared_ptr<std::map<std::string, std::string>> tf_to_ign, 
  std::shared_ptr<std::map<std::string, std::string>> ign_to_tf) {

    size_t queue_size = 10;

    std::map<std::string, std::pair<std::string, std::string>> type_to_type;
    type_to_type["bool"] = std::make_pair<std::string, std::string>("std_msgs/Bool","ignition.msgs.Boolean");
    type_to_type["color"] = std::make_pair<std::string, std::string>("std_msgs/ColorRGBA","ignition.msgs.Color");
    type_to_type["empty"] = std::make_pair<std::string, std::string>("std_msgs/Empty","ignition.msgs.Empty");
    type_to_type["int32"] = std::make_pair<std::string, std::string>("std_msgs/Int32","ignition.msgs.Int32");
    type_to_type["float32"] = std::make_pair<std::string, std::string>("std_msgs/Float32","ignition.msgs.Float");
    type_to_type["float64"] = std::make_pair<std::string, std::string>("std_msgs/Float64","ignition.msgs.Double");
    type_to_type["header"] = std::make_pair<std::string, std::string>("std_msgs/Header","ignition.msgs.Header");
    type_to_type["string"] = std::make_pair<std::string, std::string>("std_msgs/String","ignition.msgs.StringMsg");
    type_to_type["quaternion"] = std::make_pair<std::string, std::string>("geometry_msgs/Quaternion","ignition.msgs.Quaternion");
    type_to_type["vector3"] = std::make_pair<std::string, std::string>("geometry_msgs/Vector3","ignition.msgs.Vector3d");
    type_to_type["point"] = std::make_pair<std::string, std::string>("geometry_msgs/Point","ignition.msgs.Vector3d");
    type_to_type["pose"] = std::make_pair<std::string, std::string>("geometry_msgs/Pose","ignition.msgs.Pose");
    type_to_type["pose_array"] = std::make_pair<std::string, std::string>("geometry_msgs/PoseArray","ignition.msgs.Pose_V");
    type_to_type["pose_stamped"] = std::make_pair<std::string, std::string>("geomery_msgs/PoseStamped","ignition.msgs.Pose");
    type_to_type["transform"] = std::make_pair<std::string, std::string>("geometry_msgs/Transform","ignition.msgs.Pose");
    type_to_type["transform_stamped"] = std::make_pair<std::string, std::string>("geometry_msgs/TransformStamped","ignition.msgs.Pose");
    type_to_type["twist"] = std::make_pair<std::string, std::string>("geometry_msgs/Twist","ignition.msgs.Twist");
    type_to_type["actuators"] = std::make_pair<std::string, std::string>("nav_msgs/Actuators","ignition.msgs.Actuators");
    type_to_type["occupancy_grid"] = std::make_pair<std::string, std::string>("nav_msgs/OccupancyGrid","ignition.msgs.OccupanyGrid");
    type_to_type["odometry"] = std::make_pair<std::string, std::string>("nav_msgs/Odometry","ignition.msgs.Odometry");
    type_to_type["clock"] = std::make_pair<std::string, std::string>("rosgraph_msgs/Clock", "ignition.msgs.Clock");
    type_to_type["battery_state"] = std::make_pair<std::string, std::string>("sensor_msgs/BatteryState","ignition.msgs.BatteryState");
    type_to_type["camera_info"] = std::make_pair<std::string, std::string>("sensor_msgs/CameraInfo","ignition.msgs.CameraInfo");
    type_to_type["fluid_pressure"] = std::make_pair<std::string, std::string>("sensor_msgs/FluidPressure","ignition.msgs.FluidPressure");
    type_to_type["imu"] = std::make_pair<std::string, std::string>("sensor_msgs/Imu","ignition.msgs.IMU");
    type_to_type["image"] = std::make_pair<std::string, std::string>("sensor_msgs/Image","ignition.msgs.Image");
    type_to_type["joint_state"] = std::make_pair<std::string, std::string>("sensor_msgs/JointState","ignition.msgs.Model");
    type_to_type["laser_scan"] = std::make_pair<std::string, std::string>("sensor_msgs/LaserScan","ignition.msgs.LaserScan");
    type_to_type["magnetic_field"] = std::make_pair<std::string, std::string>("sensor_msgs/MagneticField","ignition.msgs.Magnetometer");
    type_to_type["nav_sat_fix"] = std::make_pair<std::string, std::string>("sensor_msgs/NavSatFix","ignition.msgs.NavSat");
    type_to_type["point_cloud2"] = std::make_pair<std::string, std::string>("sensor_msgs/PointCloud2","ignition.msgs.PointCloudPacked");
    type_to_type["tf_message"] = std::make_pair<std::string, std::string>("tf_msgs/TFMessage","ignition.msgs.Pose");
    type_to_type["marker"] = std::make_pair<std::string, std::string>("visualization_msgs/Marker","ignition.msgs.Marker");
    type_to_type["marker_array"] = std::make_pair<std::string, std::string>("visualization_msgs/MarkerArray","ignition.msgs.Marker_V");

    XmlRpc::XmlRpcValue topics;
    XmlRpc::XmlRpcValue frames;

    if (frame_available) {
      nh_private.getParam("frames", frames);

      for (uint i = 0; i < frames.size(); i++) {
        XmlRpc::XmlRpcValue it = frames[i];
        std::string ign_frame;
        std::string tf_frame;

        if (!it.hasMember("ign_frame")) {
          ROS_ERROR("Missing ign frame, skipping");
          continue;
        }
        if (!it.hasMember("tf_frame")) {
          ROS_ERROR("Missing ign frame, skipping");
          continue;
        }
        ign_frame = static_cast<std::string>(it["ign_frame"]);
        tf_frame = static_cast<std::string>(it["tf_frame"]);

        (*tf_to_ign)[tf_frame] = ign_frame;
        (*ign_to_tf)[ign_frame] = tf_frame;

        //TODO use for translation

      }
    } 

    if (topics_available) {
      nh_private.getParam("topics", topics);

      for (uint i = 0; i < topics.size(); i++) {
        XmlRpc::XmlRpcValue it = topics[i];
        std::string ign_topic;
        std::string ros_topic;
        std::string type;

        if (it.hasMember("ign_topic")) {
          ign_topic = static_cast<std::string>(it["ign_topic"]);
          if (!it.hasMember("ros_topic")) {
            ROS_WARN("No ros topic given, treating ros and ign topic as the same");
            ros_topic = ign_topic;
          }
        }
        if (it.hasMember("ros_topic")) {
          ros_topic = static_cast<std::string>(it["ros_topic"]);
          if (!it.hasMember("ign_topic")) {
            ROS_WARN("No ign topic given, treating ros and ign topic as the same");
            ign_topic = ros_topic;
          }
        }

        if (!it.hasMember("ros_topic") && !it.hasMember("ign_topic")) {
          ROS_ERROR("No topic given in current config entry, skipping");
          continue;
        }

        if (!it.hasMember("type")) {
          ROS_ERROR("No topic type given, unable to bridge this topic");
          continue;
        }
        type = static_cast<std::string>(it["type"]);

        std::string ros_type = type_to_type[type].first;
        std::string ign_type = type_to_type[type].second;

        std::cout << "creating bridge for " << ros_topic << std::endl;
        try
        {
          if (it.hasMember("direction")) {
            if (static_cast<std::string>(it["direction"]) == "to_ros") {
              ign_to_ros_handles.push_back(
              ros_ign_bridge::create_bridge_from_ign_to_ros(
                ign_node, ros_node,
                ign_type, ign_topic, queue_size,
                ros_type, ros_topic, queue_size,
                tf_to_ign, ign_to_tf));
            std::cout << "created bridge to ros" << std::endl;
                continue;
            }
            if (static_cast<std::string>(it["direction"]) == "to_ign") {
              ros_to_ign_handles.push_back(
                ros_ign_bridge::create_bridge_from_ros_to_ign(
                  ros_node, ign_node,
                  ros_type, ros_topic, queue_size,
                  ign_type, ign_topic, queue_size,
                  tf_to_ign, ign_to_tf));
            std::cout << "created bridge to ign" << std::endl;
                  continue;
            }
            else {
              bidirectional_handles.push_back(
                ros_ign_bridge::create_bidirectional_bridge(
                  ros_node, ign_node,
                  ros_type, ign_type,
                  ros_topic, ign_topic, 
                  tf_to_ign, ign_to_tf, queue_size));
            std::cout << "created bridge bidirectional" << std::endl;
                  continue;
            }
          }
          else {
            ROS_WARN("No direction given, assuming bidirectional");
            bidirectional_handles.push_back(
                ros_ign_bridge::create_bidirectional_bridge(
                  ros_node, ign_node,
                  ros_type, ign_type,
                  ros_topic, ign_topic, 
                  tf_to_ign, ign_to_tf, queue_size));
            std::cout << "created bridge bidirectional" << std::endl;
                  continue;
          }
        } catch (std::runtime_error &_e) {
          ROS_ERROR_STREAM("Failed to create a bridge for topics ["
            << ros_topic << " " << ign_topic << "] "
            << "with ROS type [" << ros_type << "] and "
            << "Ignition Transport type [" << ign_type << "]"
            << std::endl);
        }
      }
    }
    return 0;
  }
//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  // ROS node
  ros::init(argc, argv, "ros_ign_bridge");
  ros::NodeHandle ros_node;
  ros::NodeHandle nh_private("~");

  bool topics_available = false;
  bool frames_available = false;
  topics_available = nh_private.hasParam("topics");
  frames_available = nh_private.hasParam("frames");

  if (argc < 2 && !(topics_available))
  {
    usage();
    return -1;
  } 
 
  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  std::list<ros_ign_bridge::BridgeHandles> bidirectional_handles;
  std::list<ros_ign_bridge::BridgeIgnToRosHandles> ign_to_ros_handles;
  std::list<ros_ign_bridge::BridgeRosToIgnHandles> ros_to_ign_handles;

  std::shared_ptr<std::map<std::string, std::string>> tf_to_ign = std::make_shared<std::map<std::string, std::string>>();
  std::shared_ptr<std::map<std::string, std::string>> ign_to_tf = std::make_shared<std::map<std::string, std::string>>();



  if (argc > 2) {
    int res = parseArguments(argc, argv, bidirectional_handles, ign_to_ros_handles, ros_to_ign_handles, ign_node, ros_node, tf_to_ign, ign_to_tf);
    if (res != 0)
      return res;
  }
  
  if (topics_available || frames_available) {
    int res = parseParameters(topics_available, frames_available, nh_private, bidirectional_handles, ign_to_ros_handles, ros_to_ign_handles, ign_node, ros_node, tf_to_ign, ign_to_tf);
    if (res != 0) 
      return res;
  }


  // ROS asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
