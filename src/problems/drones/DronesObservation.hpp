/** @file DronesObservation.hpp
 *
 * Defines the DronesObservation class, which represents an observation in the Drones POMDP.
 */
#ifndef DRONES_OBSERVATION_HPP_
#define DRONES_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"


namespace drones {
class DronesModel;

/** A class representing an observation in the Drones POMDP.
 *
 * This includes an observation of the robot's own position, and a boolean flag representing
 * whether or not the robot sees the opponent (and hence is on the same grid square).
 */
class DronesObservation : public solver::Point {
    friend class DronesTextSerializer;
  public:
    /** Constructs a new DronesObservation for the given robot position; seesOpponent should be true
     * iff the robot sees the opponent due to being on the same square.
     */
    DronesObservation(SwarmVec myswarmposition, bool isLanded);

    virtual ~DronesObservation() = default;
    _NO_COPY_OR_MOVE(DronesObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    // double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;

    /** Returns the position the robot has observed itself in. */
    // GridPosition getPosition() const;
    SwarmVec getSwarmVecPosition() const;
    bool getIsLanded() const;
    /** Returns true iff the robot sees the opponent in the same square it is in. */
    // bool seesOpponent() const;

  private:

    SwarmVec swarmposition_;
    bool isLanded_;
};
} /* namespace drones */
#endif /* DRONES_OBSERVATION_HPP_ */
