/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_map_conversions.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/map_connection.hpp"
#include "adore_ros2_msgs/msg/map_lane.hpp"
#include "adore_ros2_msgs/msg/map_point.hpp"
#include "adore_ros2_msgs/msg/map_road.hpp"
#include "adore_ros2_msgs/msg/route.hpp"

#include <rclcpp/type_adapter.hpp>

namespace rclcpp
{

// Adapter for MapPoint
template<>
struct TypeAdapter<adore::map::MapPoint, adore_ros2_msgs::msg::MapPoint>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::MapPoint;
  using ros_message_type = adore_ros2_msgs::msg::MapPoint;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

// Adapter for MapLane
template<>
struct TypeAdapter<adore::map::Lane, adore_ros2_msgs::msg::MapLane>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::Lane;
  using ros_message_type = adore_ros2_msgs::msg::MapLane;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

// Adapter for MapConnection
template<>
struct TypeAdapter<adore::map::Connection, adore_ros2_msgs::msg::MapConnection>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::Connection;
  using ros_message_type = adore_ros2_msgs::msg::MapConnection;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

// Adapter for MapRoad
template<>
struct TypeAdapter<adore::map::Road, adore_ros2_msgs::msg::MapRoad>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::Road;
  using ros_message_type = adore_ros2_msgs::msg::MapRoad;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

// Adapter for Map
template<>
struct TypeAdapter<adore::map::Map, adore_ros2_msgs::msg::Map>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::Map;
  using ros_message_type = adore_ros2_msgs::msg::Map;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

// Adapter for RouteSection
template<>
struct TypeAdapter<adore::map::RouteSection, adore_ros2_msgs::msg::RouteSection>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::RouteSection;
  using ros_message_type = adore_ros2_msgs::msg::RouteSection;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

// Adapter for Route
template<>
struct TypeAdapter<adore::map::Route, adore_ros2_msgs::msg::Route>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::map::Route;
  using ros_message_type = adore_ros2_msgs::msg::Route;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::map::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::map::conversions::to_cpp_type( src );
  }
};

} // namespace rclcpp

namespace adore
{
using MapPointAdapter     = rclcpp::TypeAdapter<map::MapPoint, adore_ros2_msgs::msg::MapPoint>;
using LaneAdapter         = rclcpp::TypeAdapter<map::Lane, adore_ros2_msgs::msg::MapLane>;
using RoadAdapter         = rclcpp::TypeAdapter<map::Road, adore_ros2_msgs::msg::MapRoad>;
using RouteSectionAdapter = rclcpp::TypeAdapter<map::RouteSection, adore_ros2_msgs::msg::RouteSection>;

using ConnectionAdapter = rclcpp::TypeAdapter<map::Connection, adore_ros2_msgs::msg::MapConnection>;
using RouteAdapter      = rclcpp::TypeAdapter<map::Route, adore_ros2_msgs::msg::Route>;
using MapAdapter        = rclcpp::TypeAdapter<map::Map, adore_ros2_msgs::msg::Map>;
} // namespace adore