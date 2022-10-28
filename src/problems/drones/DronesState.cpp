/** @file DronesState.cpp
 *
 * Contains the implementation for the methods of DronesState.
 */
#include "DronesState.hpp"

#include <cstddef>                      // for size_t

#include <functional>   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream>
#include <vector>

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

namespace drones {
DronesState::DronesState(SwarmVec swarmPos, bool isLanded) :
    solver::Vector(),
    swarmPos_(swarmPos),
    isLanded_(isLanded){
}

DronesState::DronesState(DronesState const &other) :
        DronesState(other.swarmPos_, other.isLanded_) {
}

std::unique_ptr<solver::Point> DronesState::copy() const {
    return std::make_unique<DronesState>(swarmPos_, isLanded_);
}

bool DronesState::equals(solver::State const &otherState) const {
    DronesState const &otherDronesState = static_cast<DronesState const &>(otherState);
    return (swarmPos_ == otherDronesState.swarmPos_ && isLanded_ == otherDronesState.isLanded_ ); 
}

std::size_t DronesState::hash() const { 
    std::size_t hashValue = 0;
    for (unsigned int iter = 0; iter < swarmPos_.size(); iter++){
        tapir::hash_combine(hashValue, swarmPos_.at(iter).i);
        tapir::hash_combine(hashValue, swarmPos_.at(iter).j);
    }
    tapir::hash_combine(hashValue, isLanded_);
    return hashValue;
}

std::vector<double> DronesState::asVector() const {
    std::vector<double> vec(swarmPos_.size()*2+1);
    for (unsigned int iter = 0; iter < swarmPos_.size(); iter++){
        vec[iter*2] = swarmPos_.at(iter).i;
        vec[iter*2+1] = swarmPos_.at(iter).j;
    }
    vec.push_back(isLanded_);
    
    return vec;
}

void DronesState::print(std::ostream &os) const {
    for(auto const& iter: swarmPos_){
        os << iter << " ";
    }
    os << "isLanded: " << isLanded_;
}

bool DronesState::getIsLanded() const {
    return isLanded_;
}


SwarmVec DronesState::getSwarmVecPosition() const {
    return swarmPos_;
}


} /* namespace drones */
