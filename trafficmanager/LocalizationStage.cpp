
#include "carla/trafficmanager/Constants.h"

#include "carla/trafficmanager/LocalizationStage.h"

namespace carla {
namespace traffic_manager {

using namespace constants::PathBufferUpdate;
using namespace constants::LaneChange;
using namespace constants::WaypointSelection;

/////MTS Extension
using namespace constants::MTSCar;


LocalizationStage::LocalizationStage(
  const std::vector<ActorId> &vehicle_id_list,
  BufferMap &buffer_map,
  const SimulationState &simulation_state,
  TrackTraffic &track_traffic,
  const LocalMapPtr &local_map,
  Parameters &parameters,
  std::vector<ActorId>& marked_for_removal,
  LocalizationFrame &output_array,
  cc::DebugHelper &debug_helper,
  RandomGeneratorMap &random_devices)
  : vehicle_id_list(vehicle_id_list),
    buffer_map(buffer_map),
    simulation_state(simulation_state),
    track_traffic(track_traffic),
    local_map(local_map),
    parameters(parameters),
    marked_for_removal(marked_for_removal),
    output_array(output_array),
    debug_helper(debug_helper),
    random_devices(random_devices) {}

void LocalizationStage::Update(const unsigned long index) {

  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location vehicle_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D heading_vector = simulation_state.GetHeading(actor_id);
  const cg::Vector3D vehicle_velocity_vector = simulation_state.GetVelocity(actor_id);
  const float vehicle_speed = vehicle_velocity_vector.Length();

  // Speed dependent waypoint horizon length.
  float horizon_length = std::min(vehicle_speed * HORIZON_RATE + MINIMUM_HORIZON_LENGTH, MAXIMUM_HORIZON_LENGTH);
  const float horizon_square = SQUARE(horizon_length);

  if (buffer_map.find(actor_id) == buffer_map.end()) {
    buffer_map.insert({actor_id, Buffer()});
  }

  Buffer &waypoint_buffer = buffer_map.at(actor_id);

  // Clear buffer if vehicle is too far from the first waypoint in the buffer.
  if (!waypoint_buffer.empty() &&
      cg::Math::DistanceSquared(waypoint_buffer.front()->GetLocation(),
                                vehicle_location) > SQUARE(MAX_START_DISTANCE)) {

    auto number_of_pops = waypoint_buffer.size();
    for (uint64_t j = 0u; j < number_of_pops; ++j) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
    }
  }

  bool is_at_junction_entrance = false;

  if (!waypoint_buffer.empty()) {
    // Purge passed waypoints.
    float dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
    while (dot_product <= 0.0f && !waypoint_buffer.empty()) {

      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      if (!waypoint_buffer.empty()) {
        dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
      }
    }

    if (!waypoint_buffer.empty()) {
      // Determine if the vehicle is at the entrance of a junction.
      SimpleWaypointPtr look_ahead_point = GetTargetWaypoint(waypoint_buffer, JUNCTION_LOOK_AHEAD).first;
      is_at_junction_entrance = !waypoint_buffer.front()->CheckJunction() && look_ahead_point->CheckJunction();
      if (is_at_junction_entrance
          // Exception for roundabout in Town03.
          && local_map->GetMapName() == "Town03"
          && vehicle_location.SquaredLength() < SQUARE(30)) {
        is_at_junction_entrance = false;
      }
    }

    // Purge waypoints too far from the front of the buffer.
    while (!is_at_junction_entrance
           && !waypoint_buffer.empty()
           && waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) > horizon_square) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer, false);
    }
  }

  // Initializing buffer if it is empty.
  if (waypoint_buffer.empty()) {
    SimpleWaypointPtr closest_waypoint = local_map->GetWaypoint(vehicle_location);
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, closest_waypoint);
  }

  // Assign a lane change.


  /////MTS TODO


  const ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(actor_id);
  bool force_lane_change = lane_change_info.change_lane;
  bool lane_change_direction = lane_change_info.direction;

  if (!force_lane_change) {
    float perc_keep_right = parameters.GetKeepRightPercentage(actor_id);
    if (perc_keep_right >= 0.0f && perc_keep_right >= random_devices.at(actor_id).next()) {
      force_lane_change = true;
      lane_change_direction = true;
    }
  }

  const SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
  const float lane_change_distance = SQUARE(std::max(10.0f * vehicle_speed, INTER_LANE_CHANGE_DISTANCE));

  bool recently_not_executed_lane_change = last_lane_change_location.find(actor_id) == last_lane_change_location.end();
  bool done_with_previous_lane_change = true;
  if (!recently_not_executed_lane_change) {
    float distance_frm_previous = cg::Math::DistanceSquared(last_lane_change_location.at(actor_id), vehicle_location);
    done_with_previous_lane_change = distance_frm_previous > lane_change_distance;
  }
  bool auto_or_force_lane_change = parameters.GetAutoLaneChange(actor_id) || force_lane_change;
  bool front_waypoint_not_junction = !front_waypoint->CheckJunction();

  if (auto_or_force_lane_change
      && front_waypoint_not_junction
      && (recently_not_executed_lane_change || done_with_previous_lane_change)) {

    SimpleWaypointPtr change_over_point = AssignLaneChange(actor_id, vehicle_location, vehicle_speed,
                                                           force_lane_change, lane_change_direction);

    if (change_over_point != nullptr) {
      if (last_lane_change_location.find(actor_id) != last_lane_change_location.end()) {
        last_lane_change_location.at(actor_id) = vehicle_location;
      } else {
        last_lane_change_location.insert({actor_id, vehicle_location});
      }
      auto number_of_pops = waypoint_buffer.size();
      for (uint64_t j = 0u; j < number_of_pops; ++j) {
        PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      }
      PushWaypoint(actor_id, track_traffic, waypoint_buffer, change_over_point);
    }
  }

  // Populating the buffer.
  while (waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) <= horizon_square) {

    SimpleWaypointPtr furthest_waypoint = waypoint_buffer.back();
    std::vector<SimpleWaypointPtr> next_waypoints = furthest_waypoint->GetNextWaypoint();
    uint64_t selection_index = 0u;
    // Pseudo-randomized path selection if found more than one choice.
    if (next_waypoints.size() > 1) {
      // Arranging selection points from right to left.
      std::sort(next_waypoints.begin(), next_waypoints.end(),
                [&furthest_waypoint](const SimpleWaypointPtr &a, const SimpleWaypointPtr &b) {
                  float a_x_product = DeviationCrossProduct(furthest_waypoint->GetLocation(),
                                                            furthest_waypoint->GetForwardVector(),
                                                            a->GetLocation());
                  float b_x_product = DeviationCrossProduct(furthest_waypoint->GetLocation(),
                                                            furthest_waypoint->GetForwardVector(),
                                                            b->GetLocation());
                  return a_x_product < b_x_product;
                });
      double r_sample = random_devices.at(actor_id).next();
      double s_bucket = 100.0 / next_waypoints.size();
      selection_index = static_cast<uint64_t>(std::floor(r_sample/s_bucket));
    } else if (next_waypoints.size() == 0) {
      if (!parameters.GetOSMMode()) {
        std::cout << "This map has dead-end roads, please change the set_open_street_map parameter to true" << std::endl;
      }
      marked_for_removal.push_back(actor_id);
      break;
    }
    SimpleWaypointPtr next_wp_selection = next_waypoints.at(selection_index);
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, next_wp_selection);
  }

  ExtendAndFindSafeSpace(actor_id, is_at_junction_entrance, waypoint_buffer);

  // Editing output array
  LocalizationData &output = output_array.at(index);
  output.is_at_junction_entrance = is_at_junction_entrance;

  if (is_at_junction_entrance) {
    const SimpleWaypointPair &safe_space_end_points = vehicles_at_junction_entrance.at(actor_id);
    output.junction_end_point = safe_space_end_points.first;
    output.safe_point = safe_space_end_points.second;
  } else {
    output.junction_end_point = nullptr;
    output.safe_point = nullptr;
  }

  // Updating geodesic grid position for actor.
  track_traffic.UpdateGridPosition(actor_id, waypoint_buffer);


  /////MTS Extension

  MTS_SurroundingUpdate(index);
  MTS_RegionUpdate(index);
  if (actor_id == vehicle_id_list.at(0))
  {
    DrawLeader(actor_id, output);
    DrawNeighbor(actor_id, output);
    DrawRegion(actor_id, output);
  }

}

void LocalizationStage::ExtendAndFindSafeSpace(const ActorId actor_id,
                                               const bool is_at_junction_entrance,
                                               Buffer &waypoint_buffer) {

  SimpleWaypointPtr junction_end_point = nullptr;
  SimpleWaypointPtr safe_point_after_junction = nullptr;

  if (is_at_junction_entrance
      && vehicles_at_junction_entrance.find(actor_id) == vehicles_at_junction_entrance.end()) {

    bool entered_junction = false;
    bool past_junction = false;
    bool safe_point_found = false;
    SimpleWaypointPtr current_waypoint = nullptr;
    SimpleWaypointPtr junction_begin_point = nullptr;
    float safe_distance_squared = SQUARE(SAFE_DISTANCE_AFTER_JUNCTION);

    // Scanning existing buffer points.
    for (unsigned long i = 0u; i < waypoint_buffer.size() && !safe_point_found; ++i) {
      current_waypoint = waypoint_buffer.at(i);
      if (!entered_junction && current_waypoint->CheckJunction()) {
        entered_junction = true;
        junction_begin_point = current_waypoint;
      }
      if (entered_junction && !past_junction && !current_waypoint->CheckJunction()) {
        past_junction = true;
        junction_end_point = current_waypoint;
      }
      if (past_junction && junction_end_point->DistanceSquared(current_waypoint) > safe_distance_squared) {
        safe_point_found = true;
        safe_point_after_junction = current_waypoint;
      }
    }

    // Extend buffer if safe point not found.
    if (!safe_point_found) {
      while (!past_junction) {
        current_waypoint = current_waypoint->GetNextWaypoint().front();
        PushWaypoint(actor_id, track_traffic, waypoint_buffer, current_waypoint);
        if (!current_waypoint->CheckJunction()) {
          past_junction = true;
          junction_end_point = current_waypoint;
        }
      }

      while (!safe_point_found) {
        std::vector<SimpleWaypointPtr> next_waypoints = current_waypoint->GetNextWaypoint();
        if ((junction_end_point->DistanceSquared(current_waypoint) > safe_distance_squared)
            || next_waypoints.size() > 1
            || current_waypoint->CheckJunction()) {

          safe_point_found = true;
          safe_point_after_junction = current_waypoint;
        } else {
          current_waypoint = next_waypoints.front();
          PushWaypoint(actor_id, track_traffic, waypoint_buffer, current_waypoint);
        }
      }
    }

    if (junction_begin_point->DistanceSquared(junction_end_point) < SQUARE(MIN_JUNCTION_LENGTH)) {
      junction_end_point = nullptr;
      safe_point_after_junction = nullptr;
    }

    vehicles_at_junction_entrance.insert({actor_id, {junction_end_point, safe_point_after_junction}});
  }
  else if (!is_at_junction_entrance
           && vehicles_at_junction_entrance.find(actor_id) != vehicles_at_junction_entrance.end()) {

    vehicles_at_junction_entrance.erase(actor_id);
  }
}

void LocalizationStage::RemoveActor(ActorId actor_id) {
    last_lane_change_location.erase(actor_id);
    vehicles_at_junction.erase(actor_id);
}

void LocalizationStage::Reset() {
  last_lane_change_location.clear();
  vehicles_at_junction.clear();
}

SimpleWaypointPtr LocalizationStage::AssignLaneChange(const ActorId actor_id,
                                                      const cg::Location vehicle_location,
                                                      const float vehicle_speed,
                                                      bool force, bool direction) {

  // Waypoint representing the new starting point for the waypoint buffer
  // due to lane change. Remains nullptr if lane change not viable.
  SimpleWaypointPtr change_over_point = nullptr;

  // Retrieve waypoint buffer for current vehicle.
  const Buffer &waypoint_buffer = buffer_map.at(actor_id);

  // Check buffer is not empty.
  if (!waypoint_buffer.empty()) {
    // Get the left and right waypoints for the current closest waypoint.
    const SimpleWaypointPtr &current_waypoint = waypoint_buffer.front();
    const SimpleWaypointPtr left_waypoint = current_waypoint->GetLeftWaypoint();
    const SimpleWaypointPtr right_waypoint = current_waypoint->GetRightWaypoint();

    // Retrieve vehicles with overlapping waypoint buffers with current vehicle.
    const auto blocking_vehicles = track_traffic.GetOverlappingVehicles(actor_id);

    // Find immediate in-lane obstacle and check if any are too close to initiate lane change.
    bool obstacle_too_close = false;
    float minimum_squared_distance = std::numeric_limits<float>::infinity();
    ActorId obstacle_actor_id = 0u;
    for (auto i = blocking_vehicles.begin();
         i != blocking_vehicles.end() && !obstacle_too_close && !force;
         ++i) {
      const ActorId &other_actor_id = *i;
      // Find vehicle in buffer map and check if it's buffer is not empty.
      if (buffer_map.find(other_actor_id) != buffer_map.end() && !buffer_map.at(other_actor_id).empty()) {
        const Buffer &other_buffer = buffer_map.at(other_actor_id);
        const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
        const cg::Location other_location = other_current_waypoint->GetLocation();

        const cg::Vector3D reference_heading = current_waypoint->GetForwardVector();
        cg::Vector3D reference_to_other = other_location - current_waypoint->GetLocation();
        const cg::Vector3D other_heading = other_current_waypoint->GetForwardVector();

        WaypointPtr current_raw_waypoint = current_waypoint->GetWaypoint();
        WaypointPtr other_current_raw_waypoint = other_current_waypoint->GetWaypoint();
        // Check both vehicles are not in junction,
        // Check if the other vehicle is in front of the current vehicle,
        // Check if the two vehicles have acceptable angular deviation between their headings.
        if (!current_waypoint->CheckJunction()
            && !other_current_waypoint->CheckJunction()
            && other_current_raw_waypoint->GetRoadId() == current_raw_waypoint->GetRoadId()
            && other_current_raw_waypoint->GetLaneId() == current_raw_waypoint->GetLaneId()
            && cg::Math::Dot(reference_heading, reference_to_other) > 0.0f
            && cg::Math::Dot(reference_heading, other_heading) > MAXIMUM_LANE_OBSTACLE_CURVATURE) {
          float squared_distance = cg::Math::DistanceSquared(vehicle_location, other_location);
          // Abort if the obstacle is too close.
          if (squared_distance > SQUARE(MINIMUM_LANE_CHANGE_DISTANCE)) {
            // Remember if the new vehicle is closer.
            if (squared_distance < minimum_squared_distance && squared_distance < SQUARE(MAXIMUM_LANE_OBSTACLE_DISTANCE)) {
              minimum_squared_distance = squared_distance;
              obstacle_actor_id = other_actor_id;
            }
          } else {
            obstacle_too_close = true;
          }
        }
      }
    }

    // If a valid immediate obstacle found.
    if (!obstacle_too_close && obstacle_actor_id != 0u && !force) {
      const Buffer &other_buffer = buffer_map.at(obstacle_actor_id);
      const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
      const auto other_neighbouring_lanes = {other_current_waypoint->GetLeftWaypoint(),
                                             other_current_waypoint->GetRightWaypoint()};

      // Flags reflecting whether adjacent lanes are free near the obstacle.
      bool distant_left_lane_free = false;
      bool distant_right_lane_free = false;

      // Check if the neighbouring lanes near the obstructing vehicle are free of other vehicles.
      bool left_right = true;
      for (auto &candidate_lane_wp : other_neighbouring_lanes) {
        if (candidate_lane_wp != nullptr &&
            track_traffic.GetPassingVehicles(candidate_lane_wp->GetId()).size() == 0) {

          if (left_right)
            distant_left_lane_free = true;
          else
            distant_right_lane_free = true;
        }
        left_right = !left_right;
      }

      // Based on what lanes are free near the obstacle,
      // find the change over point with no vehicles passing through them.
      if (distant_right_lane_free && right_waypoint != nullptr
          && track_traffic.GetPassingVehicles(right_waypoint->GetId()).size() == 0) {
        change_over_point = right_waypoint;
      } else if (distant_left_lane_free && left_waypoint != nullptr
               && track_traffic.GetPassingVehicles(left_waypoint->GetId()).size() == 0) {
        change_over_point = left_waypoint;
      }
    } else if (force) {
      if (direction && right_waypoint != nullptr) {
        change_over_point = right_waypoint;
      } else if (!direction && left_waypoint != nullptr) {
        change_over_point = left_waypoint;
      }
    }

    if (change_over_point != nullptr) {
      const float change_over_distance = cg::Math::Clamp(1.5f * vehicle_speed, 3.0f, 20.0f);
      const auto starting_point = change_over_point;
      while (change_over_point->DistanceSquared(starting_point) < SQUARE(change_over_distance) &&
             !change_over_point->CheckJunction()) {
        change_over_point = change_over_point->GetNextWaypoint()[0];
      }
    }
  }

  return change_over_point;
}

void LocalizationStage::DrawBuffer(Buffer &buffer) {
  uint64_t buffer_size = buffer.size();
  uint64_t step_size =  buffer_size/20u;
  cc::DebugHelper::Color color {0u, 0u, 0u};
  cg::Location two_meters_up = cg::Location(0.0f, 0.0f, 2.0f);
  for (uint64_t i = 0u; i + step_size < buffer_size; i += step_size) {
    if (!buffer.at(i)->CheckJunction() && !buffer.at(i + step_size)->CheckJunction()) {
      color.g = 255u;
    }
    debug_helper.DrawLine(buffer.at(i)->GetLocation() + two_meters_up,
                          buffer.at(i + step_size)->GetLocation() + two_meters_up,
                          0.2f, color, 0.05f);
    color = {0u, 0u, 0u};
  }
}


/////MTS Extension

void LocalizationStage::MTS_Update(const unsigned long index)
{
  LocalizationData &output = output_array.at(index);
  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(actor_location);
  SimpleWaypointPtr actor_left_waypoint = actor_waypoint->GetLeftWaypoint();
  SimpleWaypointPtr actor_right_waypoint = actor_waypoint->GetRightWaypoint();
  
  MTS_SurroundingUpdate(index);
}


void LocalizationStage::MTS_SurroundingUpdate(const unsigned long index)
{
  const ActorId actor_id = vehicle_id_list.at(index);
  
  //Basic actor information
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  float actor_half_length = simulation_state.GetDimensions(actor_id).x;
  float actor_half_width = simulation_state.GetDimensions(actor_id).y;

  //Define search bound (Radius of circle)
  const float bound = actor_half_length + MAX_OBSERVING_DISTANCE;

  //The distance will be shorter after finding new target
  float leader_distance = FLT_MAX;
  float left_distance = FLT_MAX;
  float right_distance = FLT_MAX;
  float left_front_distance = FLT_MAX;
  float right_front_distance = FLT_MAX;
  float left_rear_distance = FLT_MAX;
  float right_rear_distance = FLT_MAX;

  //Temporary result
  boost::optional<ActorId> potential_leader; // c++17: std, but c++14: boost instead.
  boost::optional<ActorId> main_leader;
  boost::optional<ActorId> leftVeh;
  boost::optional<ActorId> rightVeh;
  boost::optional<ActorId> leftFrontVeh;
  boost::optional<ActorId> rightFrontVeh;
  boost::optional<ActorId> leftRearVeh;
  boost::optional<ActorId> rightRearVeh;

  
  //Road information
  LocalRoadInfo mid_info, left_info, right_info;
  SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(actor_location);
  SimpleWaypointPtr left_lane_waypoint = actor_waypoint->GetLeftWaypoint();
  SimpleWaypointPtr right_lane_waypoint = actor_waypoint->GetRightWaypoint();
  
  bool hasLeftLane = false;
  bool hasRightLane = false;
  bool isJunction = actor_waypoint->CheckJunction();

  
  crd::RoadId actor_road_id;
  double actor_road_length;
  if(!isJunction){
    const crd::Lane& actor_lane = local_map->GetLane(actor_waypoint);
    actor_road_id = actor_lane.GetRoad()->GetId();
    actor_road_length = actor_lane.GetRoad()->GetLength();
    GetLocalRoadInfo(mid_info, actor_lane);
    if(left_lane_waypoint != nullptr){
      hasLeftLane = true;
      const crd::Lane& left_lane = local_map->GetLeftLane(actor_waypoint);
      GetLocalRoadInfo(left_info, left_lane);
    }
    if(right_lane_waypoint != nullptr){
      hasRightLane = true;
      const crd::Lane& right_lane = local_map->GetRightLane(actor_waypoint);
      GetLocalRoadInfo(right_info, right_lane);
    }
  }

  //Scan the whole vehicle list
  for(ActorId target_id: vehicle_id_list)
  {
    //Filter: current referenced vehicle
    if(target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    const cg::Vector3D target_heading = simulation_state.GetHeading(target_id);
    float dot_heading = VectorDotProduct(actor_heading, target_heading);

    //Filter: vehicles are on opposite lane
    if(dot_heading < 0.0f)
      continue;
    
    cg::Location target_local_location = target_location;
    simulation_state.GlobalToLocal(actor_id, target_local_location);
    float target_local_location_x = target_local_location.x; 
    float target_local_location_y = target_local_location.y;
    float target_distance = actor_location.Distance(target_location);

    //Filter: vehicles are out of bounds
    if(target_distance > bound){
      continue;
    }
    bool isFront = target_local_location_x > 0.0f ? true : false ; //REAR is false
    bool isLongitudinalOverlapped = std::abs(target_local_location_x) < (actor_half_length) ? true : false;
    
    WaypointPtr target_waypoint = local_map->GetWaypoint(target_location)->GetWaypoint();
    crd::RoadId target_road_id = target_waypoint->GetRoadId();
    crd::SectionId target_section_id = target_waypoint->GetSectionId();
    crd::LaneId target_lane_id = target_waypoint->GetLaneId();
    RoadSectionPair key_pair = std::make_pair(target_road_id, target_section_id);
    
    if(mid_info.find(key_pair) != mid_info.end() && mid_info.at(key_pair) == target_lane_id){
      if(isFront && target_distance < leader_distance){
        potential_leader = main_leader;
        main_leader = target_id;
        leader_distance = target_distance;
      }
    }
    else if(hasLeftLane && left_info.find(key_pair) != left_info.end() && left_info.at(key_pair) == target_lane_id){
      if(isLongitudinalOverlapped){ 
        if(target_distance < left_distance){
          leftVeh = target_id;
          left_distance = target_distance;
        }
      }
      else{
        if(isFront){
          if(target_distance < left_front_distance){
            leftFrontVeh = target_id;
            left_front_distance = target_distance;
          }
        }
        else{
          if(target_distance < left_rear_distance){
            leftRearVeh = target_id;
            left_rear_distance = target_distance;
          }
        }
      }
    }
    else if(hasRightLane && right_info.find(key_pair) != right_info.end() && right_info.at(key_pair) == target_lane_id){
      if(isLongitudinalOverlapped){ 
        if(target_distance < right_distance){
          rightVeh = target_id;
          right_distance = target_distance;
        }
      }
      else{
        if(isFront){
          if(target_distance < right_front_distance){
            rightFrontVeh = target_id;
            right_front_distance = target_distance;
          }
        }
        else{
          if(target_distance < right_rear_distance){
            rightRearVeh = target_id;
            right_rear_distance = target_distance;
          }
        }
      }
    }  
  }//for-loop

  //Store final result
  LocalizationData &output = output_array.at(index);
  output.leader.MainLeader = main_leader;
  output.leader.PotentialLeader = potential_leader;
  output.neighbor.LeftVehicle = leftVeh;
  output.neighbor.RightVehicle = rightVeh;
  output.neighbor.LeftFrontVehicle = leftFrontVeh;
  output.neighbor.RightFrontVehicle = rightFrontVeh;
  output.neighbor.LeftRearVehicle = leftRearVeh;
  output.neighbor.RightRearVehicle = rightRearVeh;

}

void LocalizationStage::GetLocalRoadInfo(LocalRoadInfo& info, const crd::Lane& lane)
{  
  //Add current info
  crd::LaneId lane_id = lane.GetId();
  crd::SectionId section_id = lane.GetLaneSection()->GetId();
  crd::RoadId road_id = lane.GetRoad()->GetId();
  info.insert({{std::make_pair(road_id, section_id), lane_id}});
  
  bool isOtherRoad = false;
  crd::RoadId flag_road_id = road_id;

  //Add next info 
  std::vector<crd::Lane*> next_lanes = lane.GetNextLanes();
  while(!next_lanes.empty() && next_lanes.size() == 1)
  {
    crd::Lane* next_lane = next_lanes.at(0);
    crd::LaneId next_lane_id = next_lane->GetId();
    crd::SectionId next_section_id = next_lane->GetLaneSection()->GetId();
    crd::RoadId next_road_id = next_lane->GetRoad()->GetId();
    if(next_road_id != flag_road_id)
    {
      if(isOtherRoad)
        break;
      isOtherRoad = true;
      flag_road_id = next_road_id;
    }
    info.insert({{std::make_pair(next_road_id, next_section_id), next_lane_id}});
    next_lanes = next_lane->GetNextLanes();
  }
  
  isOtherRoad = false;
  flag_road_id = road_id;

  //Add Previous info
  std::vector<crd::Lane*> previous_lanes = lane.GetPreviousLanes();
  while(!previous_lanes.empty() && previous_lanes.size() == 1)
  {
    crd::Lane* previous_lane = previous_lanes.at(0);
    crd::LaneId previous_lane_id = previous_lane->GetId();
    crd::SectionId previous_section_id = previous_lane->GetLaneSection()->GetId();
    crd::RoadId previous_road_id = previous_lane->GetRoad()->GetId();
    if(previous_road_id != flag_road_id)
    {
      if(isOtherRoad)
        break;
      isOtherRoad = true;
      flag_road_id = previous_road_id;
    }
    info.insert({{std::make_pair(previous_road_id, previous_section_id), previous_lane_id}});
    previous_lanes = previous_lane->GetPreviousLanes();
  }
}

void LocalizationStage::DrawLeader(ActorId actor_id, LocalizationData &output)
{
  cg::Vector3D box_size(0.3f,0.3f,0.3f);
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  actor_location.z += 3.0f;
  cc::DebugHelper::Color color_ego {255u, 0u, 0u};
  debug_helper.DrawBox( cg::BoundingBox(actor_location, box_size), actor_rotation, 0.1f, color_ego, 0.01f, true);
  
  if(output.leader.MainLeader)
  {
    cg::Location first_location = simulation_state.GetLocation(output.leader.MainLeader.get());
    first_location.z += 3.0f;
    cc::DebugHelper::Color color_main_leader {0u, 255u, 0u}; //Green
    debug_helper.DrawBox(cg::BoundingBox(first_location, box_size), actor_rotation, 0.1f, color_main_leader, 0.01f, true);
    }

  if(output.leader.PotentialLeader)
  { 
    cg::Location second_location = simulation_state.GetLocation(output.leader.PotentialLeader.get());
    second_location.z += 3.0f;
    cc::DebugHelper::Color color_potential_leader {0u, 0u, 255u}; //Blue
    debug_helper.DrawBox(cg::BoundingBox(second_location, box_size), actor_rotation, 0.1f, color_potential_leader, 0.01f, true);
  }

  //Draw range line
  //const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  //cg::Vector3D actor_heading_unit = actor_heading.MakeUnitVector();
  //cg::Location actor_location_end = actor_location + cg::Location(actor_heading_unit *= MAX_OBSERVING_DISTANCE);
  //debug_helper.DrawLine(actor_location, actor_location_end, 0.1f, color_ego, 0.3f, true);
}

void LocalizationStage::DrawNeighbor(ActorId actor_id, LocalizationData &output)
{
  cg::Vector3D box_size(0.3f,0.3f,0.3f);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  
  if(output.neighbor.LeftVehicle)
  {
    cg::Location left_location = simulation_state.GetLocation(output.neighbor.LeftVehicle.get());
    left_location.z += 3.0f;
    cc::DebugHelper::Color color_left {255u, 0u, 255u}; //Magenta 洋紅
    debug_helper.DrawBox( cg::BoundingBox(left_location, box_size), actor_rotation, 0.1f, color_left, 0.3f, true);
  }

  if(output.neighbor.RightVehicle)
  { 
    cg::Location right_location = simulation_state.GetLocation(output.neighbor.RightVehicle.get());
    right_location.z += 3.0f;
    cc::DebugHelper::Color color_right {255u, 255u, 0u}; //Yelllow
    debug_helper.DrawBox( cg::BoundingBox(right_location, box_size), actor_rotation, 0.1f, color_right, 0.3f, true);
  }
  
  if(output.neighbor.RightFrontVehicle)
  { 
    cg::Location right_front_location = simulation_state.GetLocation(output.neighbor.RightFrontVehicle.get());
    right_front_location.z += 3.0f;
    cc::DebugHelper::Color color_right_front {0u, 0u, 0u}; //Black
    debug_helper.DrawBox( cg::BoundingBox(right_front_location, box_size), actor_rotation, 0.1f, color_right_front, 0.3f, true);
  }

  if(output.neighbor.LeftFrontVehicle)
  { 
    cg::Location left_front_location = simulation_state.GetLocation(output.neighbor.LeftFrontVehicle.get());
    left_front_location.z += 3.0f;
    cc::DebugHelper::Color color_left_front {0u, 255u, 255u}; //blue green
    debug_helper.DrawBox( cg::BoundingBox(left_front_location, box_size), actor_rotation, 0.1f, color_left_front, 0.3f, true);
  }

  if(output.neighbor.RightRearVehicle)
  { 
    cg::Location right_rear_location = simulation_state.GetLocation(output.neighbor.RightRearVehicle.get());
    right_rear_location.z += 3.0f;
    cc::DebugHelper::Color color_right_rear {155u, 55u, 255u}; //Purple
    debug_helper.DrawBox( cg::BoundingBox(right_rear_location, box_size), actor_rotation, 0.1f, color_right_rear, 0.3f, true);
  }

  if(output.neighbor.LeftRearVehicle)
  { 
    cg::Location left_rear_location = simulation_state.GetLocation(output.neighbor.LeftRearVehicle.get());
    left_rear_location.z += 3.0f;
    cc::DebugHelper::Color color_left_rear {255u, 255u, 255u}; //White
    debug_helper.DrawBox( cg::BoundingBox(left_rear_location, box_size), actor_rotation, 0.1f, color_left_rear, 0.3f, true);
  }

}


void LocalizationStage::MTS_RegionUpdate(const unsigned long index)
{ 
  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  //+Duplcate
  float actor_half_length = simulation_state.GetDimensions(actor_id).x;

  LocalizationData &output = output_array.at(index);
  MTS_Leader leader = output.leader;
  MTS_Neighbor neighbor = output.neighbor;
  
  //Current region.
  //+Duplicate
  SimpleWaypointPtr current_waypoint = local_map->GetWaypoint(actor_location);
  // MTS_Region current_region = GetRegion(actor_id, main_leader, current_waypoint, 0.0); // middle direction = 0
  MTS_Region current_region;
  float lane_width = float(current_waypoint->GetWaypoint()->GetLaneWidth());
  float half_lane_width = lane_width / 2.0f;
  
  float gap = MAX_OBSERVING_DISTANCE;
  float maxSpeed = simulation_state.GetSpeedLimit(actor_id);
  if(leader.MainLeader)
  {
    //-Add func
    ActorId main_leader = leader.MainLeader.get();
    gap = simulation_state.GetGap(actor_id, main_leader);
    maxSpeed = std::min(simulation_state.GetVelocity(main_leader).Length(), maxSpeed); // local
  }
  current_region.gap = gap;
  current_region.maxPassingSpeed = maxSpeed;

  cg::Location target = cg::Location(gap / 2.0f + actor_half_length, 0.0f, 0.0f);
  cg::Location left_border = cg::Location(gap / 2.0f + actor_half_length, -half_lane_width, 0.0f);
  cg::Location right_border = cg::Location(gap / 2.0f + actor_half_length, half_lane_width, 0.0f);
  
  simulation_state.LocalToGlobal(actor_id, target);
  simulation_state.LocalToGlobal(actor_id, left_border);
  simulation_state.LocalToGlobal(actor_id, right_border);
  
  current_region.location = target; //now is global, need to trans to local?
  current_region.leftBorder = left_border;
  current_region.rightBorder = right_border;
  current_region.width = lane_width;
  current_region.leftBorderVehicle = neighbor.LeftVehicle; 
  current_region.rightBorderVehicle = neighbor.RightVehicle;

  //Left region.
  //+Duplicate
  SimpleWaypointPtr left_lane_waypoint = current_waypoint->GetLeftWaypoint(); 
  MTS_Region left_region;

  if(left_lane_waypoint)
  {  
    //left_region = GetRegion(actor_id, leftFrontVeh, left_lane, -1.0); // left direction = -1
    float left_gap = MAX_OBSERVING_DISTANCE;
    float left_max_speed = maxSpeed;
    if(neighbor.LeftFrontVehicle)
    {
      //-Add func
      ActorId leftFrontVeh = neighbor.LeftFrontVehicle.get();
      left_gap = simulation_state.GetGap(actor_id, leftFrontVeh);
      left_max_speed = simulation_state.GetVelocity(leftFrontVeh).Length();
    }
    left_region.gap = left_gap;
    left_region.maxPassingSpeed = left_max_speed;

    cg::Location left_target = cg::Location(left_gap / 2.0f + actor_half_length, -lane_width, 0.0f);
    cg::Location left_left_border = cg::Location(gap / 2.0f + actor_half_length, -lane_width - half_lane_width, 0.0f);
    cg::Location left_right_border = cg::Location(gap / 2.0f + actor_half_length, lane_width + half_lane_width, 0.0f);
    
    simulation_state.LocalToGlobal(actor_id, left_target);
    simulation_state.LocalToGlobal(actor_id, left_left_border);
    simulation_state.LocalToGlobal(actor_id, left_right_border);

    left_region.location = left_target; //left_lane->GetLocation(); //offset -> location
    left_region.leftBorder = left_left_border;
    left_region.rightBorder = left_right_border;
    left_region.width = lane_width;
    left_region.rightBorderVehicle = actor_id;
  }
  
  //Right region.
  //+Duplicate
  SimpleWaypointPtr right_lane_waypoint = current_waypoint->GetRightWaypoint();
  MTS_Region right_region;

  if(right_lane_waypoint)
  {  
    //right_region = GetRegion(actor_id, rightFrontVeh, right_lane, 1.0); // right direction = 1
    float right_gap = MAX_OBSERVING_DISTANCE;
    float right_max_speed = maxSpeed;

    if(neighbor.RightFrontVehicle)
    {
      //-Add func
      ActorId rightFrontVeh = neighbor.RightFrontVehicle.get();
      right_gap = simulation_state.GetGap(actor_id, rightFrontVeh);
      right_max_speed = simulation_state.GetVelocity(rightFrontVeh).Length();
    }
    right_region.gap = right_gap;
    right_region.maxPassingSpeed = right_max_speed;

    cg::Location right_target = cg::Location(right_gap / 2.0f + actor_half_length, lane_width, 0.0f);
    cg::Location right_left_border = cg::Location(gap / 2.0f + actor_half_length, -lane_width - half_lane_width, 0.0f);
    cg::Location right_right_border = cg::Location(gap / 2.0f + actor_half_length, lane_width + half_lane_width, 0.0f);
    
    simulation_state.LocalToGlobal(actor_id, right_target);
    simulation_state.LocalToGlobal(actor_id, right_left_border);
    simulation_state.LocalToGlobal(actor_id, right_right_border);

    right_region.location = right_target;//left_lane->GetLocation(); //local
    right_region.leftBorder = right_left_border;
    right_region.rightBorder = right_right_border;
    right_region.width = lane_width;
    right_region.leftBorderVehicle = actor_id;
  }

  //LocalizationData &output = output_array.at(index);
  output.situation.CurrentRegion = current_region;
  output.situation.CandidateRegions.push_back(current_region);
  output.situation.CandidateRegions.push_back(left_region);
  output.situation.CandidateRegions.push_back(right_region);
}

void LocalizationStage::DrawRegion(ActorId actor_id, LocalizationData &output)
{
  for(auto &region : output.situation.CandidateRegions)
  {
    cg::Vector3D box_size(region.gap / 2.0f, region.width / 2.0f, 0.0f);
    cg::Location location = region.location;
    cg::Rotation rotation = simulation_state.GetRotation(actor_id);
    location.z += 0.5f;
    cc::DebugHelper::Color color {255u, 0u, 0u};
    debug_helper.DrawBox(cg::BoundingBox(location, box_size), rotation, 0.1f, color, 0.3f, true);
  }
}



} // namespace traffic_manager
} // namespace carla
