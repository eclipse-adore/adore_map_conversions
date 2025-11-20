#include <cmath>
#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <vector>

#include "adore_map_conversions.hpp"
#include <adore_ros2_msgs/msg/map.hpp>
#include <adore_ros2_msgs/msg/map_connection.hpp>
#include <adore_ros2_msgs/msg/map_lane.hpp>
#include <adore_ros2_msgs/msg/map_point.hpp>
#include <adore_ros2_msgs/msg/map_road.hpp>
#include <adore_ros2_msgs/msg/route.hpp>
#include <adore_ros2_msgs/msg/route_section.hpp>

namespace
{

using adore::map::conversions::to_cpp_type;
using adore::map::conversions::to_ros_msg;

constexpr double k_eps = 1e-9;

// --------- small helpers for equality ---------

bool
map_point_equal( const adore_ros2_msgs::msg::MapPoint& a, const adore_ros2_msgs::msg::MapPoint& b )
{
  return std::abs( a.x - b.x ) < k_eps && std::abs( a.y - b.y ) < k_eps && std::abs( a.s - b.s ) < k_eps
      && std::abs( a.max_speed - b.max_speed ) < k_eps && a.parent_id == b.parent_id;
}

void
expect_map_point_equal( const adore_ros2_msgs::msg::MapPoint& a, const adore_ros2_msgs::msg::MapPoint& b )
{
  EXPECT_NEAR( a.x, b.x, k_eps );
  EXPECT_NEAR( a.y, b.y, k_eps );
  EXPECT_NEAR( a.s, b.s, k_eps );
  EXPECT_NEAR( a.max_speed, b.max_speed, k_eps );
  EXPECT_EQ( a.parent_id, b.parent_id );
}

// vector equality with fixed order
void
expect_map_point_vector_equal_ordered( const std::vector<adore_ros2_msgs::msg::MapPoint>& a,
                                       const std::vector<adore_ros2_msgs::msg::MapPoint>& b )
{
  ASSERT_EQ( a.size(), b.size() );
  for( std::size_t i = 0; i < a.size(); ++i )
  {
    expect_map_point_equal( a[i], b[i] );
  }
}

// vector equality ignoring order (used where conversion might reorder)
void
expect_map_point_vector_equal_unordered( const std::vector<adore_ros2_msgs::msg::MapPoint>& a,
                                         const std::vector<adore_ros2_msgs::msg::MapPoint>& b )
{
  ASSERT_EQ( a.size(), b.size() );

  std::vector<bool> matched( b.size(), false );

  for( const auto& pa : a )
  {
    bool found = false;
    for( std::size_t i = 0; i < b.size(); ++i )
    {
      if( !matched[i] && map_point_equal( pa, b[i] ) )
      {
        matched[i] = true;
        found      = true;
        break;
      }
    }
    EXPECT_TRUE( found );
  }
}

bool
map_connection_equal( const adore_ros2_msgs::msg::MapConnection& a, const adore_ros2_msgs::msg::MapConnection& b )
{
  return a.from_id == b.from_id && a.to_id == b.to_id && a.connection_type == b.connection_type && std::abs( a.weight - b.weight ) < k_eps;
}

void
expect_map_connection_sets_equal( const std::vector<adore_ros2_msgs::msg::MapConnection>& a,
                                  const std::vector<adore_ros2_msgs::msg::MapConnection>& b )
{
  ASSERT_EQ( a.size(), b.size() );
  std::vector<bool> matched( b.size(), false );

  for( const auto& ca : a )
  {
    bool found = false;
    for( std::size_t i = 0; i < b.size(); ++i )
    {
      if( !matched[i] && map_connection_equal( ca, b[i] ) )
      {
        matched[i] = true;
        found      = true;
        break;
      }
    }
    EXPECT_TRUE( found );
  }
}

void
expect_map_lane_equal( const adore_ros2_msgs::msg::MapLane& a, const adore_ros2_msgs::msg::MapLane& b )
{
  EXPECT_EQ( a.id, b.id );
  EXPECT_EQ( a.road_id, b.road_id );
  EXPECT_EQ( a.type, b.type );
  EXPECT_EQ( a.material, b.material );
  EXPECT_NEAR( a.speed_limit, b.speed_limit, k_eps );
  EXPECT_EQ( a.left_of_reference, b.left_of_reference );

  expect_map_point_vector_equal_ordered( a.inner_points, a.inner_points );
  expect_map_point_vector_equal_ordered( a.outer_points, b.outer_points );
  expect_map_point_vector_equal_ordered( a.center_points, b.center_points );
}

void
expect_map_road_equal( const adore_ros2_msgs::msg::MapRoad& a, const adore_ros2_msgs::msg::MapRoad& b )
{
  EXPECT_EQ( a.id, b.id );
  EXPECT_EQ( a.name, b.name );
  EXPECT_EQ( a.one_way, b.one_way );
  EXPECT_EQ( a.category, b.category );

  ASSERT_EQ( a.lanes.size(), b.lanes.size() );

  // Compare lanes as a set keyed by lane id (order may change)
  for( const auto& lane_a : a.lanes )
  {
    auto it = std::find_if( b.lanes.begin(), b.lanes.end(), [&]( const auto& lane_b ) { return lane_b.id == lane_a.id; } );

    ASSERT_NE( it, b.lanes.end() ) << "Lane with id " << lane_a.id << " not found in round-trip MapRoad";
    expect_map_lane_equal( lane_a, *it );
  }
}

void
expect_map_equal( const adore_ros2_msgs::msg::Map& a, const adore_ros2_msgs::msg::Map& b )
{
  // bounding box
  EXPECT_NEAR( a.x_min, b.x_min, k_eps );
  EXPECT_NEAR( a.x_max, b.x_max, k_eps );
  EXPECT_NEAR( a.y_min, b.y_min, k_eps );
  EXPECT_NEAR( a.y_max, b.y_max, k_eps );

  // header frame_id is set to "world" by to_ros_msg(Map), so
  // we intentionally don't enforce equality here.

  // connections as set
  expect_map_connection_sets_equal( a.connections, b.connections );

  // roads as set keyed by road id
  ASSERT_EQ( a.roads.size(), b.roads.size() );
  for( const auto& road_a : a.roads )
  {
    auto it = std::find_if( b.roads.begin(), b.roads.end(), [&]( const auto& road_b ) { return road_b.id == road_a.id; } );
    ASSERT_NE( it, b.roads.end() ) << "Road with id " << road_a.id << " not found in round-trip Map";
    expect_map_road_equal( road_a, *it );
  }
}

void
expect_route_section_equal( const adore_ros2_msgs::msg::RouteSection& a, const adore_ros2_msgs::msg::RouteSection& b )
{
  EXPECT_NEAR( a.start_s, b.start_s, k_eps );
  EXPECT_NEAR( a.end_s, b.end_s, k_eps );
  EXPECT_NEAR( a.route_s, b.route_s, k_eps );
  EXPECT_EQ( a.lane_id, b.lane_id );
}

bool
route_center_point_equal( const adore_ros2_msgs::msg::MapPoint& a, const adore_ros2_msgs::msg::MapPoint& b )
{
  return map_point_equal( a, b );
}

void
expect_route_center_points_equal_unordered( const std::vector<adore_ros2_msgs::msg::MapPoint>& a,
                                            const std::vector<adore_ros2_msgs::msg::MapPoint>& b )
{
  expect_map_point_vector_equal_unordered( a, b );
}

void
expect_route_equal( const adore_ros2_msgs::msg::Route& a, const adore_ros2_msgs::msg::Route& b )
{
  // sections – order preserved
  ASSERT_EQ( a.sections.size(), b.sections.size() );
  for( std::size_t i = 0; i < a.sections.size(); ++i )
  {
    expect_route_section_equal( a.sections[i], b.sections[i] );
  }

  // center_points – may be reordered by center_lane map
  expect_route_center_points_equal_unordered( a.center_points, b.center_points );

  // start / goal
  EXPECT_NEAR( a.start.x, b.start.x, k_eps );
  EXPECT_NEAR( a.start.y, b.start.y, k_eps );
  EXPECT_NEAR( a.goal.x, b.goal.x, k_eps );
  EXPECT_NEAR( a.goal.y, b.goal.y, k_eps );

  // embedded map
  expect_map_equal( a.map, b.map );

  // header.frame_id is forced to "world" in to_ros_msg(Route) – ignore.
}

// --------- builders for sample messages ---------

adore_ros2_msgs::msg::MapPoint
make_sample_map_point( double x, double y, double s, double max_speed )
{
  adore_ros2_msgs::msg::MapPoint p;
  p.x         = x;
  p.y         = y;
  p.s         = s;
  p.max_speed = max_speed;
  // parent_id left at default; conversions should preserve it
  return p;
}

adore_ros2_msgs::msg::MapLane
make_sample_lane()
{
  adore_ros2_msgs::msg::MapLane lane;

  lane.inner_points.push_back( make_sample_map_point( 0.0, 0.0, 0.0, 10.0 ) );
  lane.outer_points.push_back( make_sample_map_point( 1.0, 0.0, 0.0, 10.0 ) );
  lane.center_points.push_back( make_sample_map_point( 0.5, 0.0, 0.0, 10.0 ) );

  // ids left at their default; we still check they round-trip.
  lane.type              = 1u;
  lane.material          = 2u;
  lane.speed_limit       = 13.5;
  lane.left_of_reference = true;

  return lane;
}

adore_ros2_msgs::msg::MapRoad
make_sample_road()
{
  adore_ros2_msgs::msg::MapRoad road;

  // id left default, but checked for equality.
  road.name     = "Test Road";
  road.one_way  = true;
  road.category = 3u;

  road.lanes.push_back( make_sample_lane() );

  return road;
}

adore_ros2_msgs::msg::MapConnection
make_sample_connection()
{
  adore_ros2_msgs::msg::MapConnection c;
  // from_id/to_id left default
  c.connection_type = 1u;
  c.weight          = 5.0;
  return c;
}

adore_ros2_msgs::msg::Map
make_sample_map()
{
  adore_ros2_msgs::msg::Map map_msg;

  map_msg.x_min = -10.0;
  map_msg.x_max = 10.0;
  map_msg.y_min = -5.0;
  map_msg.y_max = 5.0;

  map_msg.connections.push_back( make_sample_connection() );
  map_msg.roads.push_back( make_sample_road() );

  // header.frame_id will be overwritten to "world" in to_ros_msg(Map)
  return map_msg;
}

adore_ros2_msgs::msg::Route
make_sample_route()
{
  adore_ros2_msgs::msg::Route route_msg;

  // map
  route_msg.map = make_sample_map();

  // route sections
  adore_ros2_msgs::msg::RouteSection sec;
  sec.start_s = 0.0;
  sec.end_s   = 50.0;
  sec.route_s = 0.0;
  // lane_id left default – center_points use same default parent_id
  route_msg.sections.push_back( sec );

  // center points along lane (parent_id default matches lane_id default)
  adore_ros2_msgs::msg::MapPoint cp1 = make_sample_map_point( 0.0, 0.0, 0.0, 10.0 );
  adore_ros2_msgs::msg::MapPoint cp2 = make_sample_map_point( 5.0, 0.0, 5.0, 10.0 );
  route_msg.center_points.push_back( cp1 );
  route_msg.center_points.push_back( cp2 );

  // start / goal
  route_msg.start.x = -1.0;
  route_msg.start.y = 0.0;
  route_msg.goal.x  = 20.0;
  route_msg.goal.y  = 0.5;

  // header.frame_id will be set to "world" by to_ros_msg(Route)
  return route_msg;
}

} // namespace

// --------------------- TESTS ---------------------

TEST( AdoreMapConversions, MapPointRoundTripWithMaxSpeed )
{
  adore_ros2_msgs::msg::MapPoint msg;
  msg.x         = 1.0;
  msg.y         = 2.0;
  msg.s         = 3.0;
  msg.max_speed = 42.0;
  // parent_id default

  auto cpp_point     = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_point );

  expect_map_point_equal( msg, roundtrip_msg );
}

TEST( AdoreMapConversions, MapPointRoundTripWithoutMaxSpeedSentinel )
{
  adore_ros2_msgs::msg::MapPoint msg;
  msg.x         = 1.0;
  msg.y         = 2.0;
  msg.s         = 3.0;
  msg.max_speed = -1.0; // sentinel -> std::nullopt in C++ type

  auto cpp_point = to_cpp_type( msg );
  EXPECT_FALSE( cpp_point.max_speed.has_value() );

  auto roundtrip_msg = to_ros_msg( cpp_point );
  EXPECT_NEAR( roundtrip_msg.max_speed, -1.0, k_eps );
  EXPECT_EQ( roundtrip_msg.parent_id, msg.parent_id );
  EXPECT_NEAR( roundtrip_msg.x, msg.x, k_eps );
  EXPECT_NEAR( roundtrip_msg.y, msg.y, k_eps );
  EXPECT_NEAR( roundtrip_msg.s, msg.s, k_eps );
}

TEST( AdoreMapConversions, MapLaneRoundTripRosToCppToRos )
{
  auto msg = make_sample_lane();

  auto cpp_lane      = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_lane );

  expect_map_lane_equal( msg, roundtrip_msg );
}

TEST( AdoreMapConversions, MapConnectionRoundTripRosToCppToRos )
{
  auto msg = make_sample_connection();

  auto cpp_conn      = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_conn );

  EXPECT_TRUE( map_connection_equal( msg, roundtrip_msg ) );
}

TEST( AdoreMapConversions, MapRoadRoundTripRosToCppToRos )
{
  auto msg = make_sample_road();

  auto cpp_road      = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_road );

  expect_map_road_equal( msg, roundtrip_msg );
}

TEST( AdoreMapConversions, MapRoundTripRosToCppToRos )
{
  auto msg = make_sample_map();

  auto cpp_map       = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_map );

  expect_map_equal( msg, roundtrip_msg );
}

TEST( AdoreMapConversions, RouteSectionRoundTripRosToCppToRos )
{
  adore_ros2_msgs::msg::RouteSection msg;
  msg.start_s = 1.0;
  msg.end_s   = 5.0;
  msg.route_s = 10.0;
  // lane_id left default

  auto cpp_sec       = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_sec );

  expect_route_section_equal( msg, roundtrip_msg );
}

TEST( AdoreMapConversions, RouteRoundTripRosToCppToRos )
{
  auto msg = make_sample_route();

  auto cpp_route     = to_cpp_type( msg );
  auto roundtrip_msg = to_ros_msg( cpp_route );

  expect_route_equal( msg, roundtrip_msg );
}
