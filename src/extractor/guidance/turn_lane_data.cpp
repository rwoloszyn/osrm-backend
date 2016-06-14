#include "extractor/guidance/turn_lane_data.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

bool TurnLaneData::operator<(const TurnLaneData &other) const
{
    if (from < other.from)
        return true;
    if (from > other.from)
        return false;

    if (to < other.to)
        return true;
    if (to > other.to)
        return false;

    const constexpr char *tag_by_modifier[] = {"sharp_right",
                                               "right",
                                               "slight_right",
                                               "through",
                                               "slight_left",
                                               "left",
                                               "sharp_left",
                                               "reverse"};
    return std::find(tag_by_modifier, tag_by_modifier + 8, this->tag) <
           std::find(tag_by_modifier, tag_by_modifier + 8, other.tag);
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
