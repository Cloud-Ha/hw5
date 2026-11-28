/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>
#include <array>
#include <set>
#include <utility>
#include <vector>

Search::Search(Mothership &mothership)
    : mothership(mothership)
{
}

SearchResult Search::findShipwreck()
{
  constexpr int batchSize = 5000;
  constexpr int signalRange = 500;
  constexpr int minFollowUpSignal = 3;
  constexpr int followUpSegmentsPerDebris = 5000;
  constexpr int maxFollowUpSegments = 100000;

  struct RobotState
  {
    int position = 0;
    int lastSafeSegment = TETHER_LENGTH;
    int priority = 0;
    int followUpEnd = 0;
    std::vector<int> debris;
  };

  const int robotCount = mothership.getRobotCount();
  const int nearbyPathCount = std::min(robotCount, 3);
  std::vector<RobotState> robots(robotCount);

  for (const Location &abyss : mothership.getAbysses())
  {
    RobotState &state = robots[abyss.robot];
    state.lastSafeSegment =
        std::min(state.lastSafeSegment, abyss.segment - 1);
  }

  // Use only the first nearbyPathCount entries to avoid duplicate paths.
  const auto nearbyPaths = [robotCount](int robot)
  {
    const int right = (robot + 1 == robotCount) ? 0 : robot + 1;
    const int left = (robot == 0) ? robotCount - 1 : robot - 1;
    return std::array<int, 3>{robot, right, left};
  };

  const auto signalAt = [&](int robot, int segment)
  {
    const auto paths = nearbyPaths(robot);
    int signal = 0;

    for (int i = 0; i < nearbyPathCount; ++i)
    {
      const auto &debris = robots[paths[i]].debris;
      const auto first =
          std::lower_bound(debris.begin(), debris.end(), segment - signalRange);
      const auto last =
          std::upper_bound(first, debris.end(), segment + signalRange);
      signal += static_cast<int>(last - first);
    }

    return signal;
  };

  // The lowest score comes first. Robot ID breaks ties.
  std::set<std::pair<int, int>> pending;

  const auto enqueueRobot = [&](int robot)
  {
    RobotState &state = robots[robot];
    if (state.position >= state.lastSafeSegment)
    {
      return;
    }

    const int remainingFollowUp = std::clamp(
        state.followUpEnd - state.position, 0, maxFollowUpSegments);
    state.priority = state.position - remainingFollowUp;
    pending.emplace(state.priority, robot);
  };

  for (int robot = 0; robot < robotCount; ++robot)
  {
    robots[robot].position = mothership.getRobotPosition(robot);
    enqueueRobot(robot);
  }

  int activeRobot = -1;
  while (!pending.empty())
  {
    const int robot = pending.begin()->second;
    pending.erase(pending.begin());

    if (activeRobot != robot)
    {
      mothership.switchToRobot(robot);
      activeRobot = robot;
    }

    RobotState &state = robots[robot];
    const auto paths = nearbyPaths(robot);
    const int batchEnd =
        std::min(state.lastSafeSegment, state.position + batchSize);
    bool foundDebris = false;

    while (state.position < batchEnd)
    {
      const int distance =
          std::min(MAX_SEGMENTS_PER_DRIVE, batchEnd - state.position);
      const DriveResult result = mothership.driveActiveRobot(distance);

      // Debris or a wreck can stop the robot before the requested distance.
      state.position = mothership.getRobotPosition(robot);

      if (result.reason == DriveStopReason::ShipwreckFound)
      {
        return SearchResult{true, Location{robot, state.position}};
      }

      if (result.reason == DriveStopReason::DebrisFound)
      {
        state.debris.push_back(state.position);
        foundDebris = true;

        for (int i = 0; i < nearbyPathCount; ++i)
        {
          const int candidate = paths[i];
          const int signal = signalAt(candidate, state.position);

          if (signal >= minFollowUpSignal)
          {
            RobotState &candidateState = robots[candidate];
            const int followUpDistance = std::min(
                signal * followUpSegmentsPerDebris, maxFollowUpSegments);

            candidateState.followUpEnd = std::max(
                candidateState.followUpEnd, state.position + followUpDistance);
          }
        }
      }
    }

    enqueueRobot(robot);

    if (foundDebris)
    {
      // Refresh the neighbors after updating the active robot.
      for (int i = 1; i < nearbyPathCount; ++i)
      {
        const int neighbor = paths[i];
        pending.erase({robots[neighbor].priority, neighbor});
        enqueueRobot(neighbor);
      }
    }
  }

  return SearchResult{false, Location{}};
}