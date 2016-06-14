#ifndef OSRM_EXTRACTOR_GUIDANCE_TURN_LANE_DATA_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_TURN_LANE_DATA_HPP_

#include "util/typedefs.hpp"
#include <string>
#include <vector>

namespace osrm
{
namespace extractor
{
namespace guidance
{

struct TurnLaneData
{
    std::string tag;
    LaneID from;
    LaneID to;

    bool operator<(const TurnLaneData &other) const;
};
typedef std::vector<TurnLaneData> LaneDataVector;

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /* OSRM_EXTRACTOR_GUIDANCE_TURN_LANE_DATA_HPP_ */
