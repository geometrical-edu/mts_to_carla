
#pragma once

#include <memory>

#include "carla/client/DebugHelper.h"

#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/LocalizationUtils.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/RandomGenerator.h"
#include "carla/trafficmanager/TrackTraffic.h"
#include "carla/trafficmanager/SimulationState.h"
#include "carla/trafficmanager/Stage.h"

namespace carla {
namespace traffic_manager {

namespace cc = carla::client;

using LocalMapPtr = std::shared_ptr<InMemoryMap>;
using LaneChangeLocationMap = std::unordered_map<ActorId, cg::Location>;



/////MTS Extension
using RoadSectionPair = std::pair<crd::RoadId, crd::SectionId>;
using LocalRoadInfo = std::unordered_map<RoadSectionPair, crd::LaneId, boost::hash<RoadSectionPair>>;



/// This class has functionality to maintain a horizon of waypoints ahead
/// of the vehicle for it to follow.
/// The class is also responsible for managing lane change decisions and
/// modify the waypoint trajectory appropriately.
class LocalizationStage : Stage {
private:
  const std::vector<ActorId> &vehicle_id_list;
  BufferMap &buffer_map;
  const SimulationState &simulation_state;
  TrackTraffic &track_traffic;
  const LocalMapPtr &local_map;
  Parameters &parameters;
  // Array of vehicles marked by stages for removal.
  std::vector<ActorId>& marked_for_removal;
  LocalizationFrame &output_array;
  cc::DebugHelper &debug_helper;
  LaneChangeLocationMap last_lane_change_location;
  ActorIdSet vehicles_at_junction;
  using SimpleWaypointPair = std::pair<SimpleWaypointPtr, SimpleWaypointPtr>;
  std::unordered_map<ActorId, SimpleWaypointPair> vehicles_at_junction_entrance;
  RandomGeneratorMap &random_devices;

  SimpleWaypointPtr AssignLaneChange(const ActorId actor_id,
                                     const cg::Location vehicle_location,
                                     const float vehicle_speed,
                                     bool force, bool direction);

  void DrawBuffer(Buffer &buffer);

  void ExtendAndFindSafeSpace(const ActorId actor_id,
                              const bool is_at_junction_entrance,
                              Buffer &waypoint_buffer);


  /////MTS Extended

public:
  LocalizationStage(const std::vector<ActorId> &vehicle_id_list,
                    BufferMap &buffer_map,
                    const SimulationState &simulation_state,
                    TrackTraffic &track_traffic,
                    const LocalMapPtr &local_map,
                    Parameters &parameters,
                    std::vector<ActorId>& marked_for_removal,
                    LocalizationFrame &output_array,
                    cc::DebugHelper &debug_helper,
                    RandomGeneratorMap &random_devices);

  void Update(const unsigned long index) override;

  void RemoveActor(const ActorId actor_id) override;

  void Reset() override;


  /////MTS Extension
  
  void MTS_SurroundingUpdate(const unsigned long index);
  void GetLocalRoadInfo(LocalRoadInfo& info, const crd::Lane& lane);
  void DrawLeader(ActorId actor_id, LocalizationData &output);
  void DrawNeighbor(ActorId actor_id, LocalizationData &output);

  void MTS_RegionUpdate(const unsigned long index);
  void DrawRegion(ActorId actor_id, LocalizationData &output);
  void DrawRegionBuffer(ActorId actor_id, LocalizationData &output, Buffer &buffer);

  float ComputeBestLateralOffset(ActorId actor_id, const unsigned long index);
  bool CheckLeftSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region region, LocalizationData &localization);
  bool CheckRightSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region region, LocalizationData &localization);
  bool CheckSafety(ActorId actor_id, ActorId target_id, float moveSpeed, float moveTime, float* safeTime);
  float GetLateralTime(float desired_lateral_offset, ActorId actor_id, LocalizationData &localization);
  float GetLongitudinalTime(ActorId actor_id, LocalizationData &localization);
  float GetGapToStopLine(ActorId actor_id);

};

} // namespace traffic_manager
} // namespace carla
