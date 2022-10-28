/** @file DronesAction.cpp
 *
 * Contains the implementations for the methods of the DronesAction class.
 */
#include "DronesAction.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"

#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace drones {
DronesAction::DronesAction(ActionType actionType):
        actionType_(actionType) {
}

DronesAction::DronesAction(long code) :
        actionType_(static_cast<ActionType>(code)) {
}

std::unique_ptr<solver::Action> DronesAction::copy() const {
    return std::make_unique<DronesAction>(actionType_);
}

void DronesAction::print(std::ostream &os) const {
    switch (actionType_) {
    case ActionType::NORTH:
        os << "NORTH";
        break;
    case ActionType::SOUTH:
        os << "SOUTH";
        break;
    case ActionType::EAST:
        os << "EAST";
        break;
    case ActionType::WEST:
        os << "WEST";
        break;
    case ActionType::LAND:
        os << "LAND";
        break;
    case ActionType::HOVER:
        os << "HOVER";
        break;
    default:
        os << "ERROR-" << static_cast<long>(actionType_);
        break;
    }
}

long DronesAction::getBinNumber() const {
    return static_cast<long>(actionType_);
}
ActionType DronesAction::getActionType() const {
    return actionType_;
}
} /* namespace drones */
