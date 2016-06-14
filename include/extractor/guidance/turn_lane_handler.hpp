#ifndef OSRM_EXTRACTOR_GUIDANCE_TURN_LANE_HANDLER_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_TURN_LANE_HANDLER_HPP_

#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/turn_analysis.hpp"
#include "extractor/guidance/turn_lane_data.hpp"
#include "extractor/query_node.hpp"

#include "util/guidance/turn_lanes.hpp"
#include "util/name_table.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// Given an Intersection, the graph to access the data and  the turn lanes, the turn lane matcher
// assigns appropriate turn tupels to the different turns.
namespace lanes
{
class TurnLaneHandler
{
  public:
    typedef std::vector<TurnLaneData> LaneDataVector;

    TurnLaneHandler(const util::NodeBasedDynamicGraph &node_based_graph,
                    const util::NameTable &turn_lane_strings,
                    const std::vector<QueryNode> &node_info_list,
                    const TurnAnalysis &turn_analysis);

    Intersection assignTurnLanes(const NodeID at, const EdgeID via_edge, Intersection intersection) const;

  private:
    using LaneTupel = util::guidance::LaneTupel;
    std::unordered_map<LaneTupel, std::uint16_t> lane_tupels;

    // we need to be able to look at previous intersections to, in some cases, find the correct turn
    // lanes for a turn
    const util::NodeBasedDynamicGraph &node_based_graph;
    const util::NameTable &turn_lane_strings;
    const std::vector<QueryNode> &node_info_list;
    const TurnAnalysis &turn_analysis;

    // check whether we can handle an intersection
    bool isSimpleIntersection(const LaneDataVector &turn_lane_data,
                              const Intersection &intersection) const;

    // in case of a simple intersection, assign the lane entries
    Intersection simpleMatchTuplesToTurns(Intersection intersection,
                                          const LaneID num_lanes,
                                          const LaneDataVector &lane_data) const;

    // partition lane data into lane data relevant at current turn and at next turn
    std::pair<TurnLaneHandler::LaneDataVector, TurnLaneHandler::LaneDataVector> partitionLaneData(
        const NodeID at, LaneDataVector turn_lane_data, const Intersection &intersection) const;

    // if an intersection has a `none` value in the data, we need to transform it into appropriate
    // lane tupels
    LaneDataVector handleNoneValueAtSimpleTurn(LaneDataVector lane_data,
                                               const Intersection &intersection) const;
};

} // namespace lanes
} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_GUIDANCE_TURN_LANE_HANDLER_HPP_
