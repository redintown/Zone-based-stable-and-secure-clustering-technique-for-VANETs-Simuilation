#include "rm_manager.h"
#include <ns3/log.h>
#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RmManager");

RmManager::RmManager ()
    : m_a (0.5),
      m_b (0.5)
{
}

RmManager::RmManager (double a, double b)
    : m_a (a),
      m_b (b)
{
}

void RmManager::SetWeights (double a, double b)
{
    m_a = a;
    m_b = b;
}

double RmManager::CalculateRm (double distanceToCenter, double maxDistance, double speed, double maxSpeed) const
{
    // Equation (10): RM_n = A * (L_cn / max_{m \in C_k} {L_cm}) + B * (v_n / max_{m \in C_k} {v_m})
    double distanceTerm = 0.0;
    if (maxDistance > 1e-6) {
        distanceTerm = distanceToCenter / maxDistance;
    }

    double speedTerm = 0.0;
    if (maxSpeed > 1e-6) {
        speedTerm = speed / maxSpeed;
    }

    double rm = (m_a * distanceTerm) + (m_b * speedTerm);
    return rm;
}

std::map<uint32_t, double> RmManager::CalculateRmForActiveZoneVehicles (
    std::vector<CandidateData> &candidates,
    const Vector &zoneCenter) const
{
    std::map<uint32_t, double> rmMap;

    if (candidates.empty ()) {
        return rmMap;
    }

    // 1. Calculate distance to center for all candidates and find maximums in active zone
    double maxDistance = 0.0;
    double maxSpeed = 0.0;

    for (auto &cand : candidates) {
        cand.distanceToCenter = ZoneManager::CalculateDistanceToCenter (zoneCenter, cand.position);
        if (cand.distanceToCenter > maxDistance) {
            maxDistance = cand.distanceToCenter;
        }
        if (cand.speed > maxSpeed) {
            maxSpeed = cand.speed;
        }
    }

    // 2. Compute RM_n for each vehicle n using Equation (10)
    for (auto &cand : candidates) {
        cand.calculatedRm = CalculateRm (cand.distanceToCenter, maxDistance, cand.speed, maxSpeed);
        rmMap[cand.vehicleId] = cand.calculatedRm;

        NS_LOG_DEBUG ("Active Zone Candidate Vehicle " << cand.vehicleId
                      << ": L_cn=" << cand.distanceToCenter << " maxL=" << maxDistance
                      << " v_n=" << cand.speed << " maxV=" << maxSpeed
                      << " => RM=" << cand.calculatedRm);
    }

    return rmMap;
}

bool RmManager::SelectChWithMinRm (
    const std::vector<CandidateData> &candidates,
    uint32_t &selectedChId,
    double &minRm) const
{
    if (candidates.empty ()) {
        selectedChId = 0;
        minRm = 1.0;
        return false;
    }

    // Algorithm 2: Set RM_min = 1 and find candidate with minimum RM_n
    selectedChId = candidates[0].vehicleId;
    minRm = candidates[0].calculatedRm;

    for (const auto &cand : candidates) {
        if (cand.calculatedRm < minRm) {
            minRm = cand.calculatedRm;
            selectedChId = cand.vehicleId;
        }
    }

    NS_LOG_INFO ("Algorithm 2 Selected CH: Vehicle " << selectedChId << " with min RM=" << minRm);
    return true;
}

} // namespace ns3
