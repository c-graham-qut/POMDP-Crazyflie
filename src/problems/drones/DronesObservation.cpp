/** @file DronesObservation.cpp
 *
 * Contains the implementations for the methods of DronesObservation.
 */
#include "DronesObservation.hpp"


#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"

#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/Observation.hpp"             // for Observation

#include "DronesModel.hpp"

namespace drones {
DronesObservation::DronesObservation(SwarmVec swarmposition, bool isLanded) :
                    swarmposition_(swarmposition),
                    isLanded_(isLanded){
}
std::unique_ptr<solver::Observation>
DronesObservation::copy() const {
    return std::make_unique<DronesObservation>(swarmposition_, isLanded_);
}

bool DronesObservation::equals(
        solver::Observation const &otherObs) const {
    DronesObservation const &other =
        static_cast<DronesObservation const &>(otherObs);
    return swarmposition_ == other.swarmposition_ && isLanded_ == other.isLanded_;
}

std::size_t DronesObservation::hash() const {
    std::size_t hashValue = 0;
    for (unsigned int iter = 0; iter < swarmposition_.size(); iter++){
        tapir::hash_combine(hashValue, swarmposition_.at(iter).i);
        tapir::hash_combine(hashValue, swarmposition_.at(iter).j);
    }
    tapir::hash_combine(hashValue, isLanded_);
    return hashValue;
}

void DronesObservation::print(std::ostream &os) const {

    for(auto const& iter: swarmposition_){
        os << iter << " ";
    }
    os << "isLanded = " << isLanded_;
}

SwarmVec DronesObservation::getSwarmVecPosition() const {
    return swarmposition_;
}
bool DronesObservation::getIsLanded() const {
    return isLanded_;
}

}/* namespace drones */
