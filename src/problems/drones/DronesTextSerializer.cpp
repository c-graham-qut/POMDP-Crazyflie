/** @file DronesTextSerializer.cpp
 *
 * Contains the implementations of the serialization methods for Drones.
 */
#include "DronesTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream<>::__istream_type

#include "global.hpp"                     // for make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "DronesAction.hpp"
#include "DronesModel.hpp"
#include "DronesObservation.hpp"
#include "DronesState.hpp"                 // for DronesState

namespace solver {
class Solver;
} /* namespace solver */

namespace drones {
void saveVector(std::vector<long> values, std::ostream &os) {
    os << "(";
    for (auto it = values.begin(); it != values.end(); it++) {
        os << *it;
        if ((it + 1) != values.end()) {
            os << ", ";
        }
    }
    os << ")";
}

std::vector<long> loadVector(std::istream &is) {
    std::vector<long> values;
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');
    std::istringstream sstr(tmpStr);
    while (std::getline(sstr, tmpStr, ',')) {
        long value;
        std::istringstream(tmpStr) >> value;
        values.push_back(value);
    }
    return values;
}

DronesTextSerializer::DronesTextSerializer(solver::Solver *solver) :
    solver::Serializer(solver) {
}

/* ------------------ Saving change sequences -------------------- */
void DronesTextSerializer::saveModelChange(solver::ModelChange const &change, std::ostream &os) {
    DronesChange const &dronesChange = static_cast<DronesChange const &>(change);
    os << dronesChange.changeType;
    os << ": ";
    saveVector(std::vector<long> {dronesChange.i0, dronesChange.j0}, os);
    os << " ";
    saveVector(std::vector<long> {dronesChange.i1, dronesChange.j1}, os);
}
std::unique_ptr<solver::ModelChange> DronesTextSerializer::loadModelChange(std::istream &is) {
    std::unique_ptr<DronesChange> change = std::make_unique<DronesChange>();
    std::getline(is, change->changeType, ':');
    std::vector<long> v0 = loadVector(is);
    std::vector<long> v1 = loadVector(is);
    change->i0 = v0[0];
    change->j0 = v0[1];
    change->i1 = v1[0];
    change->j1 = v1[1];
    return std::move(change);
}

void DronesTextSerializer::saveState(solver::State const *state, std::ostream &os) {
    DronesState const &dronesState = static_cast<DronesState const &>(*state);
    os << dronesState.swarmPos_.size() << " ";
    for (unsigned int iter = 0; iter < dronesState.swarmPos_.size(); iter++){
        os << dronesState.swarmPos_.at(iter) << " ";
    }
    os << dronesState.isLanded_ << " ";
    
       // << dronesState.opponentPos_.i << " " << dronesState.opponentPos_.j;
    // std::cout << "here" << std::endl;
}

std::unique_ptr<solver::State> DronesTextSerializer::loadState(std::istream &is) {

    SwarmVec swarmPos;
    GridPosition swarmmember;
    long NumOfSwarmMembers, iter;
    bool isLanded;
    is >> NumOfSwarmMembers;
    while (iter < NumOfSwarmMembers) {
        is >> swarmmember;
        // std::cout << "currently reading : " << swarm1 << std::endl;
        swarmPos.push_back(swarmmember);
        iter++;
    }
    is >> isLanded;

    // std::cout << "loaded state: " << swarmPos.at(0) << swarmPos.at(1) << isLanded << unusedBool << std::endl; 

    return std::make_unique<DronesState>(swarmPos, isLanded); // This function is only invoked before the first simulation step.
}


void DronesTextSerializer::saveObservation(solver::Observation const *obs,
        std::ostream &os) {
    // std::cout << "until here saveObservsation" << std::endl;
    if (obs == nullptr) {
        os << "()";
    } else {
        DronesObservation const &observation = static_cast<DronesObservation const &>(
                *obs);
        os << observation.swarmposition_.size() << " ";
        for (unsigned int iter = 0; iter < observation.swarmposition_.size(); iter++){
            os << observation.swarmposition_.at(iter) << " ";
            }
    os << observation.isLanded_;
    }
    
}

std::unique_ptr<solver::Observation> DronesTextSerializer::loadObservation(
        std::istream &is) {

    SwarmVec swarmPos;
    GridPosition swarmmember;
    long NumOfSwarmMembers, iter;
    bool isLanded;
    is >> NumOfSwarmMembers;
    if (NumOfSwarmMembers == 0) {
        return nullptr;
    }
    while (iter < NumOfSwarmMembers) {
        is >> swarmmember;
        // std::cout << "currently reading : " << swarm1 << std::endl;
        swarmPos.push_back(swarmmember);
        iter++;
    }
    is >> isLanded;

    return std::make_unique<DronesObservation>(swarmPos, isLanded);
}


void DronesTextSerializer::saveAction(solver::Action const *action,
        std::ostream &os) {
    // std::cout << "until here saveAction" << std::endl;
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    DronesAction const &a =
            static_cast<DronesAction const &>(*action);
    ActionType code = a.getActionType();
    switch (code) {
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
        os << "ERROR-" << static_cast<long>(code);
        break;
    }
}

std::unique_ptr<solver::Action> DronesTextSerializer::loadAction(
        std::istream &is) {
    std::string text;
    is >> text;
    // std::cout << text << "until here loadAction" << std::endl;
    if (text == "NULL") {
        return nullptr;
    } else if (text == "NORTH") {
        return std::make_unique<DronesAction>(ActionType::NORTH);
    } else if (text == "SOUTH") {
        return std::make_unique<DronesAction>(ActionType::SOUTH);
    } else if (text == "EAST") {
        return std::make_unique<DronesAction>(ActionType::EAST);
    } else if (text == "WEST") {
        return std::make_unique<DronesAction>(ActionType::WEST);
    } else if (text == "LAND") {
        return std::make_unique<DronesAction>(ActionType::LAND);
    } else if (text == "HOVER") {
        return std::make_unique<DronesAction>(ActionType::HOVER);
    } else {
        std::string tmpStr;
        std::istringstream sstr(text);
        std::getline(sstr, tmpStr, '-');
        long code;
        sstr >> code;
        debug::show_message("ERROR: Invalid action!");
        return std::make_unique<DronesAction>(
                static_cast<ActionType>(code));
    }
}


int DronesTextSerializer::getActionColumnWidth(){
    return 20;
}
int DronesTextSerializer::getTPColumnWidth() {
    return 0;
}
int DronesTextSerializer::getObservationColumnWidth() {
    return 20;
}
} /* namespace drones */
