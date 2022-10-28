/** @file DronesAction.hpp
 *
 * Defines the DronesAction class, which represents an action for the Drones problem, and also the
 * ActionType enumeration, which enumerates the different types of actions for Drones.
 */
#ifndef DRONES_ACTION_HPP_
#define DRONES_ACTION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace drones {

/** An enumeration of all the available actions in the Drones PODMP. */
enum class ActionType : long {
    // make formation wider
    NORTH = 0,
    SOUTH = 1,
    EAST = 2,
    WEST = 3,

    // keep formation width and move forward
    LAND = 4,
    // rearrange formation to a straight line
    HOVER = 5,
    
};

/** A class representing an action in the Drones POMDP.
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated action mapping approach (EnumeratedActionPool) to store the available actions from
 * each belief node.
 */
class DronesAction : public solver::DiscretizedPoint {
    friend class DronesTextSerializer;
  public:
    /** Constructs a new action from the given ActionType. */
    DronesAction(ActionType actionType);
    /** Constructs a new action from the given integer code. */
    DronesAction(long code);

    virtual ~DronesAction() = default;
    _NO_COPY_OR_MOVE(DronesAction);

    std::unique_ptr<solver::Action> copy() const override;
    // double distanceTo(solver::Action const &otherAction) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override;
    /** Returns the ActionType of this action. */
    ActionType getActionType() const;

  private:
    /** The ActionType for this action in the Drones POMDP. */
    ActionType actionType_;
};
} /* namespace drones */

#endif /* DRONES_ACTION_HPP_ */
