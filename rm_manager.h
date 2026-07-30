#ifndef RM_MANAGER_H
#define RM_MANAGER_H

#include "vehicle.h"
#include "zone_manager.h"
#include <vector>
#include <map>
#include <cstdint>

namespace ns3 {

/**
 * @brief Structure representing vehicle candidate data within the Active Zone of a cluster.
 */
struct CandidateData {
    uint32_t vehicleId{0};
    Vector position{0.0, 0.0, 0.0};
    double speed{0.0};
    double distanceToCenter{0.0};
    double calculatedRm{1.0};
};

/**
 * @brief Implements Equation (10) for CH Selection via Relativity Metric (RM).
 * 
 * Equation (10):
 * RM_n = A * (L_cn / max_{m \in C_k} {L_cm}) + B * (v_n / max_{m \in C_k} {v_m})
 * 
 * Where:
 * - A and B are constants such that A + B = 1 (default A = 0.5, B = 0.5 per Table 2).
 * - m is the count/index of vehicles located in the Active Zone of cluster C_k.
 * - L_cn, L_cm are Euclidean distances from vehicles to fixed geographical center points.
 * - v_n, v_m are instantaneous speeds of vehicles.
 * - Vehicle with minimum RM_n is selected as CH (Algorithm 2).
 */
class RmManager {
public:
    RmManager();
    RmManager(double a, double b);
    ~RmManager() = default;

    /**
     * @brief Sets weighting parameters A and B where A + B = 1.
     */
    void SetWeights(double a, double b);

    /**
     * @brief Gets weight factor A (distance factor weight).
     */
    double GetA() const { return m_a; }

    /**
     * @brief Gets weight factor B (velocity factor weight).
     */
    double GetB() const { return m_b; }

    /**
     * @brief Evaluates Equation (10) for a single vehicle.
     * 
     * @param distanceToCenter L_cn: distance from vehicle n to zone center
     * @param maxDistance max_{m} {L_cm}: max distance among vehicles in active zone
     * @param speed v_n: speed of vehicle n
     * @param maxSpeed max_{m} {v_m}: max speed among vehicles in active zone
     * @return Calculated Relativity Metric (RM_n)
     */
    double CalculateRm(double distanceToCenter, double maxDistance, double speed, double maxSpeed) const;

    /**
     * @brief Computes RM_n for all vehicles located in the Active Zone of a cluster.
     * 
     * @param candidates Vector of candidates in active zone
     * @param zoneCenter Fixed center point (x_c, y_c) of the zone
     * @return Map of vehicle ID to calculated RM value
     */
    std::map<uint32_t, double> CalculateRmForActiveZoneVehicles(
        std::vector<CandidateData>& candidates,
        const Vector& zoneCenter) const;

    /**
     * @brief Selects the CH with minimum RM value according to Algorithm 2.
     * 
     * @param candidates Candidates with computed RM values
     * @param selectedChId Output parameter for selected CH vehicle ID
     * @param minRm Output parameter for minimum RM value found
     * @return true if CH was successfully selected; false if candidate set was empty
     */
    bool SelectChWithMinRm(
        const std::vector<CandidateData>& candidates,
        uint32_t& selectedChId,
        double& minRm) const;

private:
    double m_a{0.5}; // Weight factor A = 0.5
    double m_b{0.5}; // Weight factor B = 0.5
};

} // namespace ns3

#endif // RM_MANAGER_H
