/** @file DronesState.hpp
 *
 * Defines the DronesState class, which represents a state of the Drones problem.
 */
#ifndef DRONESSTATE_HPP_
#define DRONESSTATE_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/VectorState.hpp"

namespace drones {
/** A class representing a state in the Drones POMDP.
 *
 * The state contains the positions of the robot and the opponent, as well as a boolean flag for
 * whether or not the opponent has been dronesged; dronesged => terminal state.
 *
 * This class also implements solver::VectorState in order to allow the state to be easily
 * converted to a vector<double>, which can then be used inside the standard R*-tree implementation
 * of StateIndex to allow spatial lookup of states.
 */


class DronesState : public solver::VectorState {
    friend class DronesTextSerializer;
  public:
    /** Constructs a new DronesState with the given positions of the robot and opponent, and the
     * given dronesged state.
     */
    DronesState(SwarmVec swarmPos, bool isLanded);


    virtual ~DronesState() = default;
    /** A copy constructor, for convenience. */
    DronesState(DronesState const &);
    /** The move constructor for DronesState is forbidden. */
    DronesState(DronesState &&) = delete;
    /** The copy assignment operator for DronesState is forbidden. */
    virtual DronesState &operator=(DronesState const &) = delete;
    /** The move assignment operator for DronesState is forbidden. */
    virtual DronesState &operator=(DronesState &&) = delete;

    std::unique_ptr<solver::Point> copy() const override;

    // double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;
    std::size_t hash() const;

    std::vector<double> asVector() const override;
    void print(std::ostream &os) const override;

    /** Returns the position of the robot. */
    // GridPosition getRobotPosition() const;
    /** Returns the position of the opponent. */
    // GridPosition getOpponentPosition() const;
    /** Returns the position of all the robots. */

    SwarmVec getSwarmVecPosition() const;
    
    bool getIsLanded() const;



  private:
    /** The position of the robot in the grid. */
    // GridPosition robotPos_;
    /** The position of the opponent in the grid. */
    // GridPosition opponentPos_;
    /** The position of the all the robots of the swarm in the grid. */
    
    SwarmVec swarmPos_;

    bool isLanded_;


    /** A flag that is true if the opponent has been dronesged. */
    // bool isDronesged_;
};
} /* namespace drones */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the DronesState class. */
template<> struct hash<drones::DronesState> {
    /** Returns the hash value for the given DronesState. */
    std::size_t operator()(drones::DronesState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* DRONESSTATE_HPP_ */
