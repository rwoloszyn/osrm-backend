#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/turn_lane_handler.hpp"
#include "extractor/guidance/turn_lane_matcher.hpp"
#include "util/simple_logger.hpp"
#include "util/typedefs.hpp"

#include <cstdint>
#include <iomanip>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

namespace lanes
{

namespace detail
{
// find turn that is closest to a given angle
std::size_t findClosestTurnIndex(const Intersection &intersection, const double angle)
{
    const auto itr = std::min_element(intersection.begin(),
                                      intersection.end(),
                                      [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                          return angularDeviation(lhs.turn.angle, angle) <
                                                 angularDeviation(rhs.turn.angle, angle);
                                      });
    return std::distance(intersection.begin(), itr);
}


std::size_t getNumberOfTurns(const Intersection &intersection)
{
    return std::count_if(intersection.begin(), intersection.end(), [](const ConnectedRoad &road) {
        return road.entry_allowed;
    });
}


std::size_t findTag(const std::string &tag, const LaneDataVector &data)
{
    return std::distance(
        data.begin(),
        std::find_if(data.begin(), data.end(), [&](const TurnLaneData &lane_data) {
            return tag == lane_data.tag;
        }));
}

} // namespace detail

TurnLaneHandler::TurnLaneHandler(const util::NodeBasedDynamicGraph &node_based_graph,
                                 const util::NameTable &turn_lane_strings,
                                 const std::vector<QueryNode> &node_info_list,
                                 const TurnAnalysis &turn_analysis)
    : node_based_graph(node_based_graph), turn_lane_strings(turn_lane_strings),
      node_info_list(node_info_list), turn_analysis(turn_analysis)
{
}

/*
    Turn lanes are given in the form of strings that closely correspond to the direction modifiers
   we use for our turn types. However, we still cannot simply perform a 1:1 assignment.

    In this function we match the turn lane strings to the actual turns.
    The input contains of a string of the format |left|through;right|right| for a setup like

    ----------
    A -^
    ----------
    B -> -v
    ----------
    C -v
    ----------

    For this setup, we coul get a set of turns of the form

    (130, turn slight right), (180, ramp straight), (320, turn sharp left)

    Here we need to augment to:
    (130, turn slight right, <3,2,0>) (180,ramp straight,<3,1,1>), and (320, turn sharp left,
   <3,1,2>)
 */
Intersection TurnLaneHandler::assignTurnLanes(const EdgeID via_edge,
                                              Intersection intersection) const
{
    const auto &data = node_based_graph.GetEdgeData(via_edge);
    const auto turn_lane_string = data.lane_string_id != INVALID_LANE_STRINGID
                                      ? turn_lane_strings.GetNameForID(data.lane_string_id)
                                      : "";
    // FIXME this is a workaround due to https://github.com/cucumber/cucumber-js/issues/417,
    // need to switch statements when fixed
    // const auto num_lanes = std::count(turn_lane_string.begin(), turn_lane_string.end(), '|') + 1;

    // going straight, due to traffic signals, we can have uncompressed geometry
    if (intersection.size() == 2 &&
        ((data.lane_string_id != INVALID_LANE_STRINGID &&
          data.lane_string_id ==
              node_based_graph.GetEdgeData(intersection[1].turn.eid).lane_string_id) ||
         angularDeviation(intersection[1].turn.angle, STRAIGHT_ANGLE) < FUZZY_ANGLE_DIFFERENCE))
    {
        return intersection;
    }

    auto countLanes = [](const std::string &turn_lane_string) {
        return boost::numeric_cast<LaneID>(
            std::count(turn_lane_string.begin(), turn_lane_string.end(), '|') + 1 +
            std::count(turn_lane_string.begin(), turn_lane_string.end(), '&'));
    };

    Intersection previous_intersection;
    EdgeID previous_id = SPECIAL_EDGEID;
    const auto previous_lane_string = [&]() -> std::string {
        /* We need to find the intersection that is located prior to via_edge.
         *
         * NODE_X  -> PREVIOUS_ID            -> NODE -> VIA_EDGE -> INTERSECTION
         * NODE_X? <- STRAIGHTMOST           <- NODE <- UTURN
         * NODE_X? -> UTURN == PREVIOUSE_ID? -> NODE -> VIA_EDGE
         *
         * To do so, we first get the intersection at NODE and find the straightmost turn from that
         * node. This will result in NODE_X. The uturn in the intersection at NODE_X should be
         * PREVIOUS_ID. To verify that find, we check the intersection using our PREVIOUS_ID
         * candidate to check the intersection at NODE for via_edge
         */
        const constexpr double COMBINE_DISTANCE_CUTOFF = 30;
        if (data.distance > COMBINE_DISTANCE_CUTOFF)
            return {};

        const auto uturn_id = intersection[0].turn.eid;
        const auto source_intersection =
            turn_analysis.getIntersection(node_based_graph.GetTarget(via_edge), uturn_id);
        // check for a straight turn

        const auto straightmost_index =
            detail::findClosestTurnIndex(source_intersection, STRAIGHT_ANGLE);
        if (angularDeviation(source_intersection[straightmost_index].turn.angle, STRAIGHT_ANGLE) >
            FUZZY_ANGLE_DIFFERENCE)
            return {};

        const auto node_x =
            node_based_graph.GetTarget(source_intersection[straightmost_index].turn.eid);
        const auto intersection_at_straight = turn_analysis.getIntersection(
            node_based_graph.GetTarget(uturn_id), source_intersection[straightmost_index].turn.eid);

        // now check that the u-turn at the given intersection connects to via-edge
        previous_id = intersection_at_straight[0].turn.eid;
        previous_intersection = turn_analysis.getIntersection(node_x, previous_id);

        const auto check_via_edge =
            previous_intersection[detail::findClosestTurnIndex(previous_intersection,
                                                               STRAIGHT_ANGLE)]
                .turn.eid;
        if (check_via_edge != via_edge)
            return {};

        const auto &previous_data = node_based_graph.GetEdgeData(previous_id);
        auto previous_string = previous_data.lane_string_id != INVALID_LANE_STRINGID
                                   ? turn_lane_strings.GetNameForID(previous_data.lane_string_id)
                                   : "";

        if (previous_string.empty())
            return previous_string;

        previous_intersection =
            turn_analysis.assignTurnTypes(node_x, previous_id, std::move(previous_intersection));

        auto previous_lane_data =
            laneDataFromString(previous_string);

        if (isSimpleIntersection(previous_lane_data, previous_intersection))
            return "";

        return previous_string;
    }();

    if (turn_lane_string.empty() && previous_lane_string.empty())
    {
        for (auto &road : intersection)
            road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

        return intersection;
    }
    LaneID num_lanes = countLanes(turn_lane_string);
    auto lane_data = laneDataFromString(turn_lane_string);

    // if we see an invalid conversion, we stop immediately
    if (!turn_lane_string.empty() && lane_data.empty())
        return intersection;

    // might be reasonable to handle multiple turns, if we know of a sequence of lanes
    // e.g. one direction per lane, if three lanes and right, through, left available
    if (!turn_lane_string.empty() && lane_data.size() == 1 && lane_data[0].tag == "none")
    {
        for (auto &road : intersection)
            road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

        return intersection;
    }

    // check whether we are at a simple intersection

    const std::size_t possible_entries = detail::getNumberOfTurns(intersection);

    if (intersection[0].entry_allowed && detail::findTag("none", lane_data) == lane_data.size() &&
        detail::findTag("left", lane_data) == lane_data.size() &&
        detail::findTag("sharp_left", lane_data) == lane_data.size() &&
        detail::findTag("reverse", lane_data) == lane_data.size() &&
        lane_data.size() + 1 == possible_entries)
    {
        intersection[0].entry_allowed = false;
    }

    if (!lane_data.empty() && canMatchTrivially(intersection, lane_data) &&
        lane_data.size() !=
            static_cast<std::size_t>(
                lane_data.back().tag != "reverse" && intersection[0].entry_allowed ? 1 : 0) +
                possible_entries &&
        intersection[0].entry_allowed && lane_data.size() == detail::findTag("none", lane_data))
    {
        lane_data.push_back({"reverse", lane_data.back().to, lane_data.back().to});
    }

    bool is_simple = isSimpleIntersection(lane_data, intersection);
    if (is_simple && !turn_lane_string.empty())
    {
        lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);
        if (canMatchTrivially(intersection, lane_data))
            return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
        else
            return intersection;
    }
    else if (turn_lane_string.empty() && !previous_lane_string.empty())
    {
        num_lanes = countLanes(previous_lane_string);
        lane_data = laneDataFromString(previous_lane_string);

        // stop on invalid lane data conversion
        if (lane_data.empty())
            return intersection;

        is_simple = isSimpleIntersection(lane_data, intersection);

        if (is_simple && !previous_lane_string.empty())
        {
            lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);
            if (canMatchTrivially(intersection, lane_data))
                return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
            else
                return intersection;
        }
        else if (!previous_lane_string.empty())
        {
            if (lane_data.size() >= detail::getNumberOfTurns(previous_intersection) &&
                previous_intersection.size() != 2)
            {
                lane_data = partitionLaneData(node_based_graph.GetTarget(previous_id),
                                              std::move(lane_data),
                                              previous_intersection)
                                .second;

                std::sort(lane_data.begin(), lane_data.end());

                // check if we were successfull in trimming
                if (lane_data.size() == detail::getNumberOfTurns(intersection) &&
                    isSimpleIntersection(lane_data, intersection))
                {
                    lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);

                    if (canMatchTrivially(intersection, lane_data))
                        return simpleMatchTuplesToTurns(
                            std::move(intersection), num_lanes, lane_data);
                    else
                        return intersection;
                }
            }
        }
    }
    else if (!turn_lane_string.empty())
    {
        /*
            We need to check whether the turn lanes have to be propagated further at some
           points.
            Some turn lanes are given on a segment prior to the one where the turn actually
           happens.
            These have to be pushed along to the segment where the data is actually used.

                      |    |           |    |
            ----------      -----------      -----



            ----------      -----------      -----
                      |    |           |    |
                      | vv |           | ^^ |
            ----------      -----------      ------
             (a)-----^
             (b)----->
             (c)-----v
            ----------      -----------      ------
                     |      |          |     |

            Both (a) and (b) are targeting not only the intersection they are at. The correct
           representation for routing is:

                      |    |           |    |
            ----------      -----------      -----



            ----------      -----------      -----
                      |    |           |    |
                      | vv |           | ^^ |
            ----------      -----------      ------
             (a)-------------------------^
             (b)----->      ---------->
             (c)-----v
            ----------      -----------      ------
                     |      |          |     |


        */
        if (lane_data.size() >= detail::getNumberOfTurns(intersection))
        {
            if (detail::findTag("merge_to_left", lane_data) != lane_data.size() ||
                detail::findTag("merge_to_right", lane_data) != lane_data.size())
                return intersection;

            lane_data = partitionLaneData(node_based_graph.GetTarget(via_edge),
                                          std::move(lane_data),
                                          intersection)
                            .first;

            // check if we were successfull in trimming
            if (lane_data.size() == detail::getNumberOfTurns(intersection) &&
                isSimpleIntersection(lane_data, intersection))
            {
                lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);

                if (canMatchTrivially(intersection, lane_data))
                    return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
                else
                    return intersection;
            }
        }
    }
    for (auto &road : intersection)
        road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

    return intersection;
}

/*
    Lanes can have the tag none. While a nice feature for visibility, it is a terrible feature
   for
   parsing. None can be part of neighboring turns, or not. We have to look at both the
   intersection
   and the lane data to see what turns we have to augment by the none-lanes
 */
LaneDataVector
TurnLaneHandler::handleNoneValueAtSimpleTurn(LaneDataVector lane_data,
                                             const Intersection &intersection) const
{
    if (intersection.empty() || lane_data.empty() || detail::findTag("none", lane_data) == lane_data.size())
    {
        return lane_data;
    }

    // FIXME all this needs to consider the number of lanes at the target to ensure that we
    // augment
    // lanes correctly, if the target lane allows for more turns
    //
    // -----------------
    //
    // -----        ----
    //  -v          |
    // -----        |
    //      |   |   |
    //
    // A situation like this would allow a right turn from the through lane.
    //
    // -----------------
    //
    // -----    --------
    //  -v      |
    // -----    |
    //      |   |
    //
    // Here, the number of lanes in the right road would not allow turns from both lanes, but
    // only
    // from the right one

    bool has_right = false;
    bool has_through = false;
    bool has_left = false;
    std::size_t connection_count = 0;
    for (const auto &road : intersection)
    {
        if (!road.entry_allowed)
            continue;

        ++connection_count;
        const auto modifier = road.turn.instruction.direction_modifier;
        has_right |= modifier == DirectionModifier::Right;
        has_right |= modifier == DirectionModifier::SlightRight;
        has_right |= modifier == DirectionModifier::SharpRight;
        has_through |= modifier == DirectionModifier::Straight;
        has_left |= modifier == DirectionModifier::Left;
        has_left |= modifier == DirectionModifier::SlightLeft;
        has_left |= modifier == DirectionModifier::SharpLeft;
    }

    if (intersection[0].entry_allowed && lane_data.back().tag != "reverse")
        --connection_count;

    const constexpr char *tag_by_modifier[] = {"reverse",
                                               "sharp_right",
                                               "right",
                                               "slight_right",
                                               "through",
                                               "slight_left",
                                               "left",
                                               "sharp_left"};

    // TODO check for impossible turns to see whether the turn lane is at the correct place

    for (std::size_t index = 0; index < lane_data.size(); ++index)
    {
        if (lane_data[index].tag == "none")
        {
            // we have to create multiple turns
            if (connection_count > lane_data.size())
            {
                // a none-turn is allowing multiple turns. we have to add a lane-data entry for
                // every possible turn. This should, hopefully, only be the case for single lane
                // entries?

                // looking at the left side first
                const auto range = [&]() {
                    if (index == 0)
                    {
                        // find first connection_count - lane_data.size() valid turns
                        std::size_t count = 0;
                        for (std::size_t intersection_index = 1;
                             intersection_index < intersection.size();
                             ++intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            if (count > connection_count - lane_data.size())
                                return std::make_pair(std::size_t{1}, intersection_index + 1);
                        }
                    }
                    else if (index + 1 == lane_data.size())
                    {
                        BOOST_ASSERT(!lane_data.empty());
                        // find last connection-count - last_data.size() valid turns
                        std::size_t count = 0;
                        for (std::size_t intersection_index = intersection.size() - 1;
                             intersection_index > 0;
                             --intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            if (count > connection_count - lane_data.size())
                                return std::make_pair(intersection_index, intersection.size());
                        }
                    }
                    else
                    {
                        // skip the first #index valid turns, find next connection_count -
                        // lane_data.size() valid ones

                        std::size_t begin = 1, count = 0, intersection_index;
                        for (intersection_index = 1; intersection_index < intersection.size();
                             ++intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            // if we reach the amount of
                            if (count >= index)
                            {
                                begin = intersection_index + 1;
                                break;
                            }
                        }

                        // reset count to find the number of necessary entries
                        count = 0;
                        for (intersection_index = begin; intersection_index < intersection.size();
                             ++intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            if (count > connection_count - lane_data.size())
                            {
                                return std::make_pair(begin, intersection_index + 1);
                            }
                        }
                    }
                    // this should, theoretically, never be reached
                    util::SimpleLogger().Write(logWARNING)
                        << "Failed lane assignment. Reached bad situation.";
                    return std::make_pair(std::size_t{0}, std::size_t{0});
                }();
                for (auto intersection_index = range.first; intersection_index < range.second;
                     ++intersection_index)
                {
                    if (intersection[intersection_index].entry_allowed)
                    {
                        // FIXME this probably can be only a subset of these turns here?
                        lane_data.push_back(
                            {tag_by_modifier[intersection[intersection_index]
                                                 .turn.instruction.direction_modifier],
                             lane_data[index].from,
                             lane_data[index].to});
                    }
                }
                lane_data.erase(lane_data.begin() + index);
                std::sort(lane_data.begin(), lane_data.end());
            }
            // we have to reduce it, assigning it to neighboring turns
            else if (connection_count < lane_data.size())
            {
                // a prerequisite is simple turns. Larger differences should not end up here
                BOOST_ASSERT(connection_count + 1 == lane_data.size());
                // an additional line at the side is only reasonable if it is targeting public
                // service vehicles. Otherwise, we should not have it
                // TODO what about lane numbering. Should we count differently?
                if (index == 0 || index + 1 == lane_data.size())
                {
                    // FIXME not augment, if this is a psv lane only
                    if (index == 0)
                    {
                        lane_data[1].from = lane_data[0].from;
                    }
                    else
                    {
                        lane_data[index - 1].to = lane_data[index].to;
                    }
                    lane_data.erase(lane_data.begin() + index);
                }
                else if (lane_data[index].to - lane_data[index].from <= 1)
                {
                    lane_data[index - 1].to = lane_data[index].from;
                    lane_data[index + 1].from = lane_data[index].to;

                    lane_data.erase(lane_data.begin() + index);
                }
            }
            // we have to rename and possibly augment existing ones. The pure count remains the
            // same.
            else
            {
                // find missing tag and augment neighboring, if possible
                if (index == 0)
                {
                    if (has_right &&
                        (lane_data.size() == 1 || (lane_data[index + 1].tag != "sharp_right" &&
                                                   lane_data[index + 1].tag != "right")))
                    {
                        lane_data[index].tag = "right";
                        if (lane_data.size() > 1 && lane_data[index + 1].tag == "through")
                        {
                            lane_data[index + 1].from = lane_data[index].from;
                            // turning right through a possible through lane is not possible
                            lane_data[index].to = lane_data[index].from;
                        }
                    }
                    else if (has_through &&
                             (lane_data.size() == 1 || lane_data[index + 1].tag != "through"))
                    {
                        lane_data[index].tag = "through";
                    }
                }
                else if (index + 1 == lane_data.size())
                {
                    if (has_left && ((lane_data[index - 1].tag != "sharp_left" &&
                                      lane_data[index - 1].tag != "left")))
                    {
                        lane_data[index].tag = "left";
                        if (lane_data[index - 1].tag == "through")
                        {
                            lane_data[index - 1].to = lane_data[index].to;
                            // turning left through a possible through lane is not possible
                            lane_data[index].from = lane_data[index].to;
                        }
                    }
                    else if (has_through && lane_data[index - 1].tag != "through")
                    {
                        lane_data[index].tag = "through";
                    }
                }
                else
                {
                    if ((lane_data[index + 1].tag == "left" ||
                         lane_data[index + 1].tag == "slight_left" ||
                         lane_data[index + 1].tag == "sharp_left") &&
                        (lane_data[index - 1].tag == "right" ||
                         lane_data[index - 1].tag == "slight_right" ||
                         lane_data[index - 1].tag == "sharp_right"))
                    {
                        lane_data[index].tag = "through";
                    }
                }
            }
            std::sort(lane_data.begin(), lane_data.end());
            break;
        }
    }

    // BOOST_ASSERT( lane_data.size() + 1 >= intersection.size() );
    return lane_data;
}

/* A simple intersection does not depend on the next intersection coming up. This is important
 * for
 * turn lanes, since traffic signals and/or segregated a intersection can influence the
 * interpretation of turn-lanes at a given turn.
 *
 * Here we check for a simple intersection. A simple intersection has a long enough segment
 * following
 * the turn, offers no straight turn, or only non-trivial turn operations.
 */
bool TurnLaneHandler::isSimpleIntersection(const LaneDataVector &lane_data,
                                           const Intersection &intersection) const
{
    if (lane_data.empty())
        return true;
    // if we are on a straight road, turn lanes are only reasonable in connection to the next
    // intersection, or in case of a merge. If not all but one (straight) are merges, we don't
    // consider the intersection simple
    if (intersection.size() == 2)
    {
        return std::count_if(
                   lane_data.begin(),
                   lane_data.end(),
                   [](const TurnLaneData &data) { return boost::starts_with(data.tag, "merge"); }) +
                   std::size_t{1} >=
               lane_data.size();
    }

    // in case an intersection offers far more lane data items than actual turns, some of them
    // have
    // to be for another intersection. A single additional item can be for an invalid bus lane.
    const auto num_turns = [&]() {
        auto count = detail::getNumberOfTurns(intersection);
        if (count < lane_data.size() && !intersection[0].entry_allowed &&
            lane_data.back().tag == "reverse")
            return count + 1;
        return count;
    }();

    // more than two additional lane data entries -> lanes target a different intersection
    if (num_turns + std::size_t{2} <= lane_data.size())
    {
        return false;
    }

    // single additional lane data entry is alright, if it is none at the side. This usually
    // refers to a bus-lane
    if (num_turns + std::size_t{1} == lane_data.size() && lane_data.front().tag != "none" &&
        lane_data.back().tag != "none")
    {
        return false;
    }

    // more turns than lane data
    if (num_turns > lane_data.size() &&
        lane_data.end() ==
            std::find_if(lane_data.begin(), lane_data.end(), [](const TurnLaneData &data) {
                return data.tag == "none";
            }))
    {
        return false;
    }

    if (num_turns > lane_data.size() && intersection[0].entry_allowed &&
        !(detail::findTag("reverse", lane_data) ||
          (lane_data.back().tag != "left" && lane_data.back().tag != "sharp_left")))
    {
        return false;
    }

    // check if we can find a valid 1:1 mapping in a straightforward manner
    bool all_simple = true;
    bool has_none = false;
    std::unordered_set<std::size_t> matched_indices;
    for (const auto &data : lane_data)
    {
        if (data.tag == "none")
        {
            has_none = true;
            continue;
        }

        const auto best_match = [&]() {
            if (data.tag != "reverse" || lane_data.size() == 1)
                return findBestMatch(data.tag, intersection);

            // lane_data.size() > 1
            if (lane_data.back().tag == "reverse")
                return findBestMatchForReverse(lane_data[lane_data.size() - 2].tag,
                                                              intersection);

            BOOST_ASSERT(lane_data.front().tag == "reverse");
            return findBestMatchForReverse(lane_data[1].tag, intersection);
        }();
        std::size_t match_index = std::distance(intersection.begin(), best_match);
        all_simple &= (matched_indices.count(match_index) == 0);
        matched_indices.insert(match_index);
        // in case of u-turns, we might need to activate them first
        all_simple &= (best_match->entry_allowed || match_index == 0);
        all_simple &= isValidMatch(data.tag, best_match->turn.instruction);
    }

    // either all indices are matched, or we have a single none-value
    if (all_simple && (matched_indices.size() == lane_data.size() ||
                       (matched_indices.size() + 1 == lane_data.size() && has_none)))
        return true;

    // better save than sorry
    return false;
}

std::pair<LaneDataVector, LaneDataVector>
TurnLaneHandler::partitionLaneData(const NodeID at,
                                   LaneDataVector turn_lane_data,
                                   const Intersection &intersection) const
{
    BOOST_ASSERT(turn_lane_data.size() >= detail::getNumberOfTurns(intersection));
    /*
     * A Segregated intersection can provide turn lanes for turns that are not yet possible.
     * The straightforward example would be coming up to the following situation:
     *         (1)             (2)
     *        | A |           | A |
     *        | | |           | ^ |
     *        | v |           | | |
     * -------     -----------     ------
     *  B ->-^                        B
     * -------     -----------     ------
     *  B ->-v                        B
     * -------     -----------     ------
     *        | A |           | A |
     *
     * Traveling on road B, we have to pass A at (1) to turn left onto A at (2). The turn
     * lane itself may only be specified prior to (1) and/or could be repeated between (1)
     * and (2). To make sure to announce the lane correctly, we need to treat the (in this
     * case left) turn lane as if it were to continue straight onto the intersection and
     * look back between (1) and (2) to make sure we find the correct lane for the left-turn.
     */

    // Try and maitch lanes to available turns. For Turns that are not directly matchable, check
    // whether we can match them at the upcoming intersection.

    const auto straightmost_index = detail::findClosestTurnIndex(intersection, STRAIGHT_ANGLE);

    BOOST_ASSERT(straightmost_index < intersection.size());
    const auto &straightmost = intersection[straightmost_index];

    // we need to be able to enter the straightmost turn
    if (!straightmost.entry_allowed)
        return {turn_lane_data, {}};

    std::vector<bool> matched_at_first(turn_lane_data.size(), false);
    std::vector<bool> matched_at_second(turn_lane_data.size(), false);

    // find out about the next intersection. To check for valid matches, we also need the turn types
    auto next_intersection = turn_analysis.getIntersection(at, straightmost.turn.eid);
    next_intersection =
        turn_analysis.assignTurnTypes(at, straightmost.turn.eid, std::move(next_intersection));

    // check where we can match turn lanes
    std::size_t straightmost_tag_index = turn_lane_data.size();
    for (std::size_t lane = 0; lane < turn_lane_data.size(); ++lane)
    {
        if (turn_lane_data[lane].tag == "none" || turn_lane_data[lane].tag == "reverse")
            continue;

        const auto best_match =
            findBestMatch(turn_lane_data[lane].tag, intersection);
        if (isValidMatch(turn_lane_data[lane].tag, best_match->turn.instruction))
        {
            matched_at_first[lane] = true;

            if (static_cast<std::size_t>(std::distance(intersection.begin(), best_match)) ==
                straightmost_index)
                straightmost_tag_index = lane;
        }

        const auto best_match_at_next_intersection =
            findBestMatch(turn_lane_data[lane].tag, next_intersection);
        if (isValidMatch(turn_lane_data[lane].tag,
                                        best_match_at_next_intersection->turn.instruction))
            matched_at_second[lane] = true;

        // we need to match all items to either the current or the next intersection
        if (!(matched_at_first[lane] || matched_at_second[lane]))
            return {turn_lane_data, {}};
    }

    auto none_index = detail::findTag("none", turn_lane_data);

    // if the turn lanes are pull forward, we might have to add an additional straight tag
    // did we find something that matches against the straightmost road?
    if (straightmost_tag_index == turn_lane_data.size())
    {
        if (none_index != turn_lane_data.size())
            straightmost_tag_index = none_index;
    }

    // TODO handle reverse

    // handle none values
    if (none_index != turn_lane_data.size())
    {
        if (static_cast<std::size_t>(
                std::count(matched_at_first.begin(), matched_at_first.end(), true)) <=
            detail::getNumberOfTurns(intersection))
            matched_at_first[none_index] = true;

        if (static_cast<std::size_t>(
                std::count(matched_at_second.begin(), matched_at_second.end(), true)) <=
            detail::getNumberOfTurns(next_intersection))
            matched_at_second[none_index] = true;
    }

    const auto augmentEntry = [&](TurnLaneData &data) {
        for (std::size_t lane = 0; lane < turn_lane_data.size(); ++lane)
            if (matched_at_second[lane])
            {
                data.from = std::min(turn_lane_data[lane].from, data.from);
                data.to = std::max(turn_lane_data[lane].to, data.to);
            }

    };

    LaneDataVector first, second;
    for (std::size_t lane = 0; lane < turn_lane_data.size(); ++lane)
    {

        if (matched_at_second[lane])
            second.push_back(turn_lane_data[lane]);

        // augment straightmost at this intersection to match all turns that happen at the next
        if (lane == straightmost_tag_index)
        {
            augmentEntry(turn_lane_data[straightmost_tag_index]);
        }

        if (matched_at_first[lane])
            first.push_back(turn_lane_data[lane]);
    }

    if (straightmost_tag_index == turn_lane_data.size() &&
        static_cast<std::size_t>(
            std::count(matched_at_second.begin(), matched_at_second.end(), true)) ==
            detail::getNumberOfTurns(next_intersection))
    {
        TurnLaneData data = {"through", 255, 0};
        augmentEntry(data);
        first.push_back(data);
        std::sort(first.begin(), first.end());
    }

    // TODO augment straightmost turn
    return {std::move(first), std::move(second)};
}

Intersection TurnLaneHandler::simpleMatchTuplesToTurns(Intersection intersection,
                                                       const LaneID num_lanes,
                                                       const LaneDataVector &lane_data) const
{
    if (lane_data.empty())
        return intersection;

    if (std::count_if(lane_data.begin(), lane_data.end(), [](const TurnLaneData &data) {
            return boost::starts_with(data.tag, "merge");
        }) > 0)
    {
        return intersection;
    }

    const std::size_t possible_entries = detail::getNumberOfTurns(intersection);

    for (auto entry : lane_data)
        if (entry.tag == "none")
        {
            util::SimpleLogger().Write(logDEBUG)
                << "Did not handle \"none\" prior to lane assignment";
            return intersection;
        }

    std::size_t valid_turn = 0;
    for (std::size_t road_index = 1;
         road_index < intersection.size() && valid_turn < possible_entries;
         ++road_index)
    {
        if (intersection[road_index].entry_allowed)
        {
            BOOST_ASSERT(lane_data[valid_turn].from != INVALID_LANEID);
            intersection[road_index].turn.instruction.lane_tupel = {
                LaneID(lane_data[valid_turn].to - lane_data[valid_turn].from + 1),
                lane_data[valid_turn].from};
            const bool uses_all_lanes =
                lane_data[valid_turn].to - lane_data[valid_turn].from + 1 == num_lanes;
            BOOST_ASSERT(findBestMatch(lane_data[valid_turn].tag, intersection) ==
                         intersection.begin() + road_index);
            ++valid_turn;
            if (TurnType::Suppressed == intersection[road_index].turn.instruction.type &&
                !uses_all_lanes)
                intersection[road_index].turn.instruction.type = TurnType::UseLane;
        }
    }

    if (valid_turn < lane_data.size())
    {
        // TODO make sure that we are not at an intersection with a middle island
        if (lane_data.back().tag == "reverse" && valid_turn + 1 == lane_data.size())
        {
            std::size_t u_turn = 0;
            if (node_based_graph.GetEdgeData(intersection[0].turn.eid).reversed)
            {
                if (intersection.back().entry_allowed ||
                    intersection.back().turn.instruction.direction_modifier !=
                        DirectionModifier::SharpLeft)
                    return intersection;
                u_turn = intersection.size() - 1;
            }
            intersection[u_turn].entry_allowed = true;
            intersection[u_turn].turn.instruction.type = TurnType::Turn;
            intersection[u_turn].turn.instruction.direction_modifier = DirectionModifier::UTurn;
            intersection[u_turn].turn.instruction.lane_tupel = {
                LaneID(lane_data.back().to - lane_data.back().from + 1), lane_data.back().from};
        }
        else
        {
            util::SimpleLogger().Write(logDEBUG) << "failed in turn lane matching.";
        }
    }
    return intersection;
}

} // namespace lanes
} // namespace guidance
} // namespace extractor
} // namespace osrm
