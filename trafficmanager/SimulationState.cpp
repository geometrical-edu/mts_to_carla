
#include "carla/trafficmanager/SimulationState.h"

namespace carla {
namespace traffic_manager {

SimulationState::SimulationState() {}

void SimulationState::AddActor(ActorId actor_id,
                               KinematicState kinematic_state,
                               StaticAttributes attributes,
                               TrafficLightState tl_state) {
  actor_set.insert(actor_id);
  kinematic_state_map.insert({actor_id, kinematic_state});
  static_attribute_map.insert({actor_id, attributes});
  tl_state_map.insert({actor_id, tl_state});
}

bool SimulationState::ContainsActor(ActorId actor_id) const {
  return actor_set.find(actor_id) != actor_set.end();
}

void SimulationState::RemoveActor(ActorId actor_id) {
  actor_set.erase(actor_id);
  kinematic_state_map.erase(actor_id);
  static_attribute_map.erase(actor_id);
  tl_state_map.erase(actor_id);
}

void SimulationState::Reset() {
  actor_set.clear();
  kinematic_state_map.clear();
  static_attribute_map.clear();
  tl_state_map.clear();
}

void SimulationState::UpdateKinematicState(ActorId actor_id, KinematicState state) {
  kinematic_state_map.at(actor_id) = state;
}

void SimulationState::UpdateTrafficLightState(ActorId actor_id, TrafficLightState state) {
  tl_state_map.at(actor_id) = state;
}

cg::Location SimulationState::GetLocation(ActorId actor_id) const {
  return kinematic_state_map.at(actor_id).location;
}

cg::Rotation SimulationState::GetRotation(ActorId actor_id) const {
  return kinematic_state_map.at(actor_id).rotation;
}

cg::Vector3D SimulationState::GetHeading(ActorId actor_id) const {
  return kinematic_state_map.at(actor_id).rotation.GetForwardVector();
}

cg::Vector3D SimulationState::GetVelocity(ActorId actor_id) const {
  return kinematic_state_map.at(actor_id).velocity;
}

float SimulationState::GetSpeedLimit(ActorId actor_id) const {
  return kinematic_state_map.at(actor_id).speed_limit;
}

bool SimulationState::IsPhysicsEnabled(ActorId actor_id) const {
  return kinematic_state_map.at(actor_id).physics_enabled;
}

TrafficLightState SimulationState::GetTLS(ActorId actor_id) const {
  return tl_state_map.at(actor_id);
}

ActorType SimulationState::GetType(ActorId actor_id) const {
  return static_attribute_map.at(actor_id).actor_type;
}

cg::Vector3D SimulationState::GetDimensions(ActorId actor_id) const {
  const StaticAttributes &attributes = static_attribute_map.at(actor_id);
  return cg::Vector3D(attributes.half_length, attributes.half_width, attributes.half_height);
}


/////MTS Extension
void SimulationState::GlobalToLocal(const ActorId actor_id, cg::Location &location) const{
  //use actor_id to get transform matrix (actor as origin of coordinate)
  cg::Transform transform(kinematic_state_map.at(actor_id).location,
                          kinematic_state_map.at(actor_id).rotation);
  transform.InverseTransformPoint(location);
}

void SimulationState::LocalToGlobal(const ActorId actor_id, cg::Location &location) const{
  cg::Transform transform(kinematic_state_map.at(actor_id).location,
                          kinematic_state_map.at(actor_id).rotation);
  transform.TransformPoint(location);
}

float SimulationState::GetGap(const ActorId actor_id, const ActorId target_id) const{
  float halfPredLen = static_attribute_map.at(target_id).half_length; //pred->getCurrentController()->getLength() / 2.0f;
	float halfVehLen = static_attribute_map.at(actor_id).half_length; //getLength() / 2.0f;
	float relativeOffset = GetRelativeOffset(actor_id, target_id);
	float gap = relativeOffset - (halfPredLen + halfVehLen);
	return gap;
}

float SimulationState::GetRelativeOffset(const ActorId actor_id, const ActorId target_id) const{
  cg::Location target_location = kinematic_state_map.at(target_id).location;
  GlobalToLocal(actor_id, target_location);
  return target_location.x;
}

float SimulationState::GetDynamicWidth(const ActorId actor_id) const{
  float yaw_angle = kinematic_state_map.at(actor_id).rotation.yaw;
  float width = static_attribute_map.at(actor_id).half_width * 2;

  if(yaw_angle == 0)
    return width;

  float length = static_attribute_map.at(actor_id).half_length * 2;
  float diagonal_length = sqrt(length * length + width * width);
  float base_angle = acos(length / diagonal_length);
  float angle_1 = std::abs(yaw_angle + base_angle);
  float angle_2 = std::abs(yaw_angle - base_angle);

  if(yaw_angle > 0)
    return diagonal_length * sin(angle_1);
  
  return diagonal_length * sin(angle_2);
}

float SimulationState::GetDynamicLength(const ActorId actor_id) const{
  float yaw_angle = kinematic_state_map.at(actor_id).rotation.yaw;
  float length = static_attribute_map.at(actor_id).half_length * 2;

  if(yaw_angle == 0)
    return length;

  float width = static_attribute_map.at(actor_id).half_width * 2;
  float diagonal_length = sqrt(length * length + width * width);
  float base_angle = acos(length / diagonal_length);
  float angle_1 = std::abs(yaw_angle + base_angle);
  float angle_2 = std::abs(yaw_angle - base_angle);

  if(yaw_angle > 0)
    return diagonal_length * cos(angle_2);
 
  return diagonal_length * cos(angle_1);
}

float SimulationState::GetLateralSeparation(const ActorId actor_id, const ActorId target_id) const{
  cg::Location target_location = kinematic_state_map.at(actor_id).location;
  GlobalToLocal(actor_id, target_location);
  return -target_location.y;
} 


} // namespace  traffic_manager
} // namespace carla
