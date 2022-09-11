/** @file DronesModel.cpp
 *
 * Contains the implementations for the core functionality of the Drones POMDP.
 */
#include "DronesModel.hpp"

#include <cmath>                        // for floor, pow
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <memory>
#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iomanip>                      // for operator<<, setw
#include <iostream>                     // for cout
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <unordered_map>                // for _Node_iterator, operator!=, unordered_map<>::iterator, _Node_iterator_base, unordered_map
#include <utility>                      // for make_pair, move, pair

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator!=, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for State, operator<<, operator==

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/indexing/FlaggingVisitor.hpp"
#include "solver/indexing/RTree.hpp"
#include "solver/indexing/SpatialIndexVisitor.hpp"             // for State, operator<<, operator==

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "DronesAction.hpp"
#include "DronesObservation.hpp"
#include "DronesOptions.hpp"
#include "DronesState.hpp"                 // for DronesState
#include "DronesTextSerializer.hpp"

using std::cout;
using std::endl;

namespace drones {
    struct pos {
        int swarmmember;
        GridPosition grid;
    };
    pos current;

    bool sortByInt(const pos &lhs, const pos &rhs) { return lhs.swarmmember < rhs.swarmmember; }

    DronesModel::DronesModel(RandomGenerator *randGen, std::unique_ptr<DronesOptions> options) :
    ModelWithProgramOptions("Drones", randGen, std::move(options)),
    options_(const_cast<DronesOptions *>(static_cast<DronesOptions const *>(getOptions()))),
    measurement_uncertainty_(options_->measurement_uncertainty),
    longitudinal_movement_uncertainty_(options_->longitudinal_movement_uncertainty),
    lateral_movement_uncertainty_(options_->lateral_movement_uncertainty),
    stepCost_(options_->stepCost),
            nRows_(0), // to be updated
            nCols_(0), // to be updated
            mapText_(), // will be pushed to
            envMap_(), // will be pushed to
            nActions_(5), // if set to a lower number the last actions will not appear as possible actions
            initialPositions_(){ // will be pushed to


    // Read the map from the file.
    std::ifstream inFile;
    inFile.open(options_->mapPath);
    if (!inFile.is_open()) {
        std::ostringstream message;
        message << "ERROR: Failed to open " << options_->mapPath;
        debug::show_message(message.str());
        std::exit(1);
    }
    inFile >> nRows_ >> nCols_;
    std::string line;
    std::vector<pos> v;

    getline(inFile, line);
    for (long row = 0; row < nRows_; row++) {
        getline(inFile, line);
        mapText_.push_back(line);
        for (long column = 0; column < line.length(); column++){
            if (isdigit(line[column])){
                current.swarmmember = line[column]- '0';
                current.grid = GridPosition(row, column);
                v.push_back(current);
                
            }
        }
    }
    inFile.close();
    std::sort(v.begin(), v.end(), sortByInt);
    for (unsigned int iter = 0; iter < v.size(); iter++){
        initialPositions_.push_back(v.at(iter).grid);
    }

    options_->numberOfStateVariables = initialPositions_.size()*2 + 1;

    initialize();
    if (options_->hasVerboseOutput) {
        cout << "InitialPositions = ";
        for (auto position : initialPositions_){
            cout << position << " ";
        }
        cout << endl;
        cout << "Constructed the DronesModel" << endl;
        cout << "Discount: " << options_->discountFactor << endl;
        cout << "Size: " << nRows_ << " by " << nCols_ << endl;
        cout << "measurement_uncertainty: " << measurement_uncertainty_ << " %" << endl;
        cout << "longitudinal_movement_uncertainty: " << longitudinal_movement_uncertainty_ << " %" << endl;
        cout << "lateral_movement_uncertainty: " << lateral_movement_uncertainty_ << " %" << endl;
        cout << "stepCost: " << stepCost_ << endl;
        cout << "maximumDepth: " << options_->maximumDepth << endl;;
        cout << "nActions: " << nActions_ << endl;
        cout << "nStVars: " << options_->numberOfStateVariables << endl;
        cout << "minParticleCount: " << options_->minParticleCount << endl;
        cout << "Environment:" << endl << endl;
        drawEnv(cout);
    }
}


void DronesModel::initialize() {
    GridPosition p;
    envMap_.resize(nRows_);
    for (p.i = nRows_ - 1; p.i >= 0; p.i--) {
        envMap_[p.i].resize(nCols_);
        for (p.j = 0; p.j < nCols_; p.j++) {
            char c = mapText_[p.i][p.j];
            DronesCellType cellType;
            if (c == 'X') {
                cellType = DronesCellType::WALL;
            } else {
                cellType = DronesCellType::EMPTY;
            }
            envMap_[p.i][p.j] = cellType;
        }
    }
}

GridPosition DronesModel::randomEmptyCell() {
    GridPosition pos;
    while (true) {
        pos.i = std::uniform_int_distribution<long>(0, nRows_ - 1)(
            *getRandomGenerator());
        pos.j = std::uniform_int_distribution<long>(0, nCols_ - 1)(
            *getRandomGenerator());
        if (envMap_[pos.i][pos.j] == DronesCellType::EMPTY) {
            break;
        }
    }
    return pos;
}


/* --------------- The model interface proper ----------------- */
std::unique_ptr<solver::State> DronesModel::sampleAnInitState() {
    return sampleStateUninformed();
}

std::unique_ptr<solver::State> DronesModel::sampleStateUninformed() {
    SwarmVec swarmPos;
    bool isLanded = false;
    swarmPos = initialPositions_;
    // cout << "size in sample = " << initialPositions_.size() << endl;
    // swarmPos.push_back(GridPosition(nRows_-2, nCols_/2-3));
    // swarmPos.push_back(GridPosition(nRows_-1,2));
    // swarmPos.push_back(GridPosition(nRows_-1,3));
    // swarmPos.push_back(GridPosition(nRows_-1,nCols_/2-3));
    // swarmPos.push_back(GridPosition(nRows_-1,nCols_/2+3));
    // swarmPos.push_back(GridPosition(nRows_-2,nCols_/2+3));

    // IMPORTANT: if the number of state variables changes (e.g. by adding members to swarmPos) the n

    return std::make_unique<DronesState>(swarmPos, isLanded);
}

bool DronesModel::isTerminal(solver::State const &state) {
    DronesState mystate = static_cast<DronesState const &>(state);
    SwarmVec swarmPos = mystate.getSwarmVecPosition();
    bool terminal = false;
    bool isLanded = mystate.getIsLanded();
    isLanded = mystate.getIsLanded();

    // only if all swarm members arrive at the top isTerminal will be true
    // for (unsigned int iter = 0; iter < swarmPos.size(); iter++){
    //     if (swarmPos.at(iter).i != 0)
    //     {
    //         terminal = false;
    //     }
    // }

    // if the POMDP chooses the action LAND then the isLanded will be changed to true which basically means the POMDP gave up
    // if (isLanded == true && unusedBool == true){
    if (isLanded == true){
    	terminal = true;
    	// cout << "the POMDP gave up: " << mystate.getIsLanded() << " "<< mystate.getUnusedBool() << endl;
    }
    return terminal;
}


// this function is only invoked when changes are applied
bool DronesModel::isValid(solver::State const &state) {
    DronesState const dronesState = static_cast<DronesState const &>(state);
    // TagState const tagState = static_cast<TagState const &>(state);
    bool loopValid = true;
    bool wasValid = true;
    for (unsigned int iter = 0; iter < dronesState.getSwarmVecPosition().size(); iter++)
    {   
      loopValid = isValid(dronesState.getSwarmVecPosition().at(iter));    
      if (!loopValid) {
    		// movedSwarmPos.at(iter) = swarmPos.at(iter);
          wasValid = false;
      }
  }
  // cout << "isValid(): state: " << dronesState << " was valid: " << wasValid << endl;
  return wasValid;
    // return true;
}


/* -------------------- Black box dynamics ---------------------- */
std::pair<std::unique_ptr<DronesState>, bool> DronesModel::makeNextState(
    solver::State const &state, solver::Action const &action) {

    DronesState const &dronesState = static_cast<DronesState const &>(state);
    DronesAction const &dronesAction = static_cast<DronesAction const &>(action);
    SwarmVec swarmPos;
    swarmPos = dronesState.getSwarmVecPosition();
    
    SwarmVec newSwarmPos;
    bool wasValid;
    bool isLanded = false;


    if (dronesAction.getActionType() == ActionType::LAND){
    	if (isLanded != true){
    	   isLanded = true;
    	}
    }else{
    	isLanded = false;
    }
    wasValid = false;
    std::tie(newSwarmPos, wasValid) = getMovedSwarmPos(swarmPos, dronesAction.getActionType());
    if (wasValid == false){
    }
    // cout << "newSwarmPos = " << newSwarmPos.at(0) << newSwarmPos.at(1) << "wasValid = " << wasValid << endl;
    return std::make_pair(std::make_unique<DronesState>(newSwarmPos, isLanded),
        wasValid);
}


// handles the movement of the swarm
std::pair<SwarmVec, bool>  DronesModel::getMovedSwarmPos(SwarmVec const &swarmPos, 
    ActionType action) {

    SwarmVec movedSwarmPos = swarmPos;
    // make all positions move right, but up/down depending on the actions (misuse North/South to control formation width)
    bool loopValid = true;
    bool wasValid = true;
    int min_row = swarmPos.at(0).i;

    // find swarm member with that is the closest to the goal
    for (unsigned int iter = 0; iter < swarmPos.size(); iter++){
        if (swarmPos.at(iter).i < min_row){
            min_row = swarmPos.at(iter).i;
        }
    }

    for (unsigned int iter = 0; iter < swarmPos.size(); iter++)
    {   

        switch (action) {
            case ActionType::WIDER:         // make formation wider
                if (swarmPos.at(iter).j > (nCols_-1)/2){
                    movedSwarmPos.at(iter).j += 1;
                }
                else if (swarmPos.at(iter).j < (nCols_-1)/2){
                    movedSwarmPos.at(iter).j -= 1;
                }
                else{                       // swarm member is in the middle column
                    movedSwarmPos.at(iter).j -= 0;
                }

                break;
        case ActionType::NARROWER:        // make formation narrower
            if (swarmPos.at(iter).j < (nCols_-1)/2){
                    movedSwarmPos.at(iter).j += 1;
                }
                else if (swarmPos.at(iter).j > (nCols_-1)/2){
                    movedSwarmPos.at(iter).j -= 1;
                }
                else{                       // swarm member is in the middle column
                    movedSwarmPos.at(iter).j -= 0;
                }

            break;
        case ActionType::FORWARD:
            movedSwarmPos.at(iter).i -= 1;
            break;
        case ActionType::LAND:
        	// does not move the drones, but sets isLanded to true
            break;
        case ActionType::HOVER:
            // do nothing
            break;
        case ActionType::REARRANGE:
            // do nothing
            if (swarmPos.at(iter).i > min_row){
                movedSwarmPos.at(iter).i -= 1;
            }
            break;
        default:
            std::ostringstream message;
            message << "Invalid action: " << (long) action;
            debug::show_message(message.str());
            break;
        } // switch


        loopValid = isValid(movedSwarmPos.at(iter));
        if (!loopValid) {
            wasValid = false;
        }
    } // for loop
    if (wasValid == false){
		// movedSwarmPos = swarmPos;
    }




// UNCERTAINTY IN MOVEMENT
    SwarmVec uncertainSwarmPos;

    // int longitudinal_movement_uncertainty = 0;  // 5
    // int lateral_movement_uncertainty = 0;       // 5

    int rand_1;
    int rand_2;
    bool coin_toss_1; // random bool between 0 and 1
    bool coin_toss_2; // random bool between 0 and 1
    GridPosition grid;
    for (unsigned int iter = 0; iter < movedSwarmPos.size(); iter++){
        grid = movedSwarmPos.at(iter);
        rand_1 = rand() % 100; // random int between 0 and 99
        rand_2 = rand() % 100; // random int between 0 and 99
        coin_toss_1 = rand() % 2; // random bool between 0 and 1
        coin_toss_2 = rand() % 2; // random bool between 0 and 1

        // if (action != ActionType::LAND && action != ActionType::HOVER && action != ActionType::REARRANGE){
        // longitudinal uncertainty
            if (rand_1 < longitudinal_movement_uncertainty_ && action == ActionType::FORWARD){
                if (coin_toss_1 > 0){
                    grid.i -= 1;
                }else{
                    grid.i += 1;
                }
            }
        // lateral uncertainty
            if (action == ActionType::NARROWER || action == ActionType::WIDER){
                if (rand_2 < lateral_movement_uncertainty_){
                    if (coin_toss_2 > 0){
                        grid.j -= 1;
                    }else{
                        grid.j += 1;
                    }
                }
            }
        uncertainSwarmPos.push_back(grid);
    }

    return std::make_pair(uncertainSwarmPos, wasValid);
}



bool DronesModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
        && position.j < nCols_ && envMap_[position.i][position.j] != DronesCellType::WALL);
}

std::unique_ptr<solver::Observation> DronesModel::makeObservation(DronesState const &nextState) {
    SwarmVec swarmPos;
    int rand_1; // random int between 0 and 99
    int rand_2; // random int between 0 and 99
    bool coin_toss_1; // random bool between 0 and 1
    bool coin_toss_2; // random bool between 0 and 1
    GridPosition grid;
    for (unsigned int iter = 0; iter < nextState.getSwarmVecPosition().size(); iter++){
    	// Choosing random positions as the next statemakes the POMDP replenish the particles for a very long time.
        // tThis is probably because most of the random states have never been discovered.
        // By choosing neighboring states (e.g. one grid field away from expected position) the POMDP can replenish the particles very fast.

        rand_1 = rand() % 100; // random int between 0 and 99
        rand_2 = rand() % 100; // random int between 0 and 99
        coin_toss_1 = rand() % 2; // random int between 0 and 1
        coin_toss_2 = rand() % 2; // random int between 0 and 1

        // neighboring states can be possible with short replenishing time
        grid = nextState.getSwarmVecPosition().at(iter);
        
        // longitudinal uncertainty
        if (rand_1 < measurement_uncertainty_){
            if (coin_toss_1 > 0){
                grid.i -= 1;
            }else{
                grid.i += 1;
            }
        }
        // lateral uncertainty
        if (rand_2 < measurement_uncertainty_){
            if (coin_toss_2 > 0){
                grid.j -= 1;
            }else{
                grid.j += 1;
            }
        }

        swarmPos.push_back(grid);
    }
    bool isLanded = nextState.getIsLanded();

    return std::make_unique<DronesObservation>(swarmPos, isLanded);
}

double DronesModel::generateReward(solver::State const &state,
    solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
    solver::State const &nextState) {
    double current_reward = 0;

    DronesState const &dronesState = static_cast<DronesState const &>(state);
    DronesState const &nextdronesState = static_cast<DronesState const &>(nextState);
    DronesAction const &dronesAction = static_cast<DronesAction const &>(action);

    /*
     current_reward += 3 * (dronesState.getSwarmVecPosition().at(dronesState.getSwarmVecPosition().size()-1).j - \
    dronesState.getSwarmVecPosition().at(0).j - nCols_ + 1);
    */
    int max_dist = dronesState.getSwarmVecPosition().at(dronesState.getSwarmVecPosition().size()-1).j - \
        dronesState.getSwarmVecPosition().at(0).j;
    // if (5 == max_dist || max_dist == 6){
    //     current_reward += 0;
    // }else{
    current_reward -= stepCost_;
    // }
    current_reward -= 0.1*(max_dist-5.5)*(max_dist-5.5);

    // if (dronesAction.getActionType() == ActionType::HOVER){
    //     current_reward -= 0.25;
    // }

    // if (dronesAction.getActionType() == ActionType::FORWARD){
    //     current_reward += 0.1*0.5*0.5;
    // }

    GridPosition position;
    GridPosition next_position;
    bool reachedGoal = true;
    int highest_row = 0;
    int lowest_row = nRows_-1;
    bool collision = false;
    for (unsigned int iter = 0; iter < dronesState.getSwarmVecPosition().size(); iter++){
        position = dronesState.getSwarmVecPosition().at(iter);
        next_position = nextdronesState.getSwarmVecPosition().at(iter);

        if (position.i < lowest_row){
            lowest_row = position.i;
        }
        if (position.i > highest_row){
            highest_row = position.i;
        }

        
        // swarm members are in the map and not in an obstacle
        if (isValid(next_position)){
    		      // do nothing
        }else{
            collision = true;
        }
        //find collisions between the swarm members

        for (unsigned int sec = 0; sec < dronesState.getSwarmVecPosition().size(); sec++){
            if (position == dronesState.getSwarmVecPosition().at(sec) && iter != sec){
                collision = true;
            }
        }

        // SwarmVec::iterator it = std::find(dronesState.getSwarmVecPosition().begin(), dronesState.getSwarmVecPosition().end(), 22);

        if (position.i > 1){
            reachedGoal= false;
        }
    } // for loop

    current_reward -= 0.1 * (highest_row - lowest_row)*(highest_row - lowest_row);

    // if (lowest_row != highest_row){
        // current_reward -= 0.1;
    // }
    if (collision == true){
        current_reward -= 11;
    }

    if (dronesAction.getActionType() == ActionType::LAND && reachedGoal == false){
		  current_reward = - 5; //must be bigger than the negative reward from having a narrow formation all the time
	} else if (dronesAction.getActionType() == ActionType::LAND && reachedGoal == true){
        current_reward += 10;
    }
    

    return current_reward;

}

std::unique_ptr<solver::State> DronesModel::generateNextState(
    solver::State const &state, solver::Action const &action,
        solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<DronesState const &>(state), action).first;
}

std::unique_ptr<solver::Observation> DronesModel::generateObservation(
        solver::State const */*state*/, solver::Action const &/*action*/,
        solver::TransitionParameters const */*tp*/,
    solver::State const &nextState) {
    return makeObservation(static_cast<DronesState const &>(nextState));
}

solver::Model::StepResult DronesModel::generateStep(solver::State const &state,
    solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action.copy();
    std::unique_ptr<DronesState> nextState = makeNextState(state, action).first;

    drones::DronesState deref = *nextState;

    solver::State & my_nextState = static_cast<solver::State &>(deref);

    result.observation = makeObservation(*nextState);
    result.reward = generateReward(state, action, nullptr, *nextState);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);

    return result;
}

solver::Model::StepResult DronesModel::generateRealStep(solver::State const &state,
    solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action.copy();
    std::unique_ptr<DronesState> nextState = makeNextState(state, action).first;

    drones::DronesState deref = *nextState;

    solver::State & my_nextState = static_cast<solver::State &>(deref);

    

    std::unique_ptr<solver::Observation> real_observation;
    bool isLanded = false;
    SwarmVec swarmPos;
    GridPosition swarmmember;
    std::ifstream pos_file;
    std::string line;
    std::stringstream sstream;

    pos_file.open("../../../problems/drones/changes/position_measurements.txt");
    if (pos_file.is_open()){
        getline(pos_file,line);
    }
    pos_file.close();
    sstream << line;
    // cout << "length = " << line.length() << endl;
    if (line.length() == 0){
        result.observation = makeObservation(*nextState); // only for the first step
        result.nextState = std::move(nextState);
    }else{
        for (int iter = 0; iter < deref.getSwarmVecPosition().size(); iter++){
            sstream >> swarmmember;
            std::cout << "currently reading : " << swarmmember << std::endl;
            swarmPos.push_back(swarmmember);
        }
        result.observation = std::make_unique<DronesObservation>(swarmPos, isLanded);
        result.nextState = std::make_unique<DronesState>(swarmPos, isLanded);
    }




    result.reward = generateReward(state, action, nullptr, *nextState);
    result.isTerminal = isTerminal(*nextState);
    

    return result;
}

/* -------------- Methods for handling model changes ---------------- */
void DronesModel::applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
    solver::Solver *solver, bool print) {
    solver::StatePool *pool = nullptr;
    if (solver != nullptr) {
        pool = solver->getStatePool();
    }

    solver::HeuristicFunction heuristic = getHeuristicFunction();
    std::vector<double> allHeuristicValues;
    if (pool != nullptr) {
        long nStates = pool->getNumberOfStates();
        allHeuristicValues.resize(nStates);
        for (long index = 0; index < nStates; index++) {
            allHeuristicValues[index] = heuristic(nullptr, pool->getInfoById(index)->getState(),
                nullptr);
        }
    }
    for (auto const &change : changes) {
        DronesChange const &dronesChange = static_cast<DronesChange const &>(*change);
        if (options_->hasVerboseOutput && print == true) {
            cout << dronesChange.changeType << " from (" << dronesChange.i0 << ", "
            << dronesChange.j0 << ")";
            cout << " to (" << dronesChange.i1 << ", " << dronesChange.j1 << ")"<< endl;
        }

        DronesCellType newCellType;
        if (dronesChange.changeType == "Add Obstacles") {
            newCellType = DronesCellType::WALL;
        } else if (dronesChange.changeType == "Remove Obstacles") {
            newCellType = DronesCellType::EMPTY;
        } else {
            cout << "Invalid change type: " << dronesChange.changeType;
            continue;
        }

        for (long i = dronesChange.i0; i <= dronesChange.i1; i++) {
            for (long j = dronesChange.j0; j <= dronesChange.j1; j++) {
                envMap_[i][j] = newCellType;
            }
        }

        if (pool == nullptr) {
            continue;
        }

        solver::RTree *tree = static_cast<solver::RTree *>(pool->getStateIndex());
        if (tree == nullptr) {
            debug::show_message("ERROR: state index must be enabled to handle changes in Drones!");
            std::exit(4);
        }

        double iLo = dronesChange.i0;
        double iHi = dronesChange.i1;
        double iMx = nRows_ - 1.0;

        double jLo = dronesChange.j0;
        double jHi = dronesChange.j1;
        double jMx = nCols_ - 1.0;

        // Adding walls => any states where the robot or the opponent are in a wall must
        // be deleted.


        // if (newCellType == DronesCellType::WALL) {
        //     solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::DELETED);
        //     // Robot is in a wall.
        //     tree->boxQuery(visitor,
        //             {iLo, jLo, 0.0, 0.0, 0.0},
        //             {iHi, jHi, iMx, jMx, 1.0});
        //     // Opponent is in a wall.
        //     tree->boxQuery(visitor,
        //             {0.0, 0.0, iLo, jLo, 0.0},
        //             {iMx, jMx, iHi, jHi, 1.0});

        // }

        // Also, state transitions around the edges of the new / former obstacle must be revised.
        solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::TRANSITION);
        tree->boxQuery(visitor,
            {iLo - 1, jLo - 1, 0.0, 0.0, 0.0},
            {iHi + 1, jHi + 1, iMx, jMx, 1.0});
        tree->boxQuery(visitor,
            {0.0, 0.0, iLo - 1, jLo - 1, 0.0},
            {iMx, jMx, iHi + 1, jHi + 1, 1.0});
    }

    // Check for heuristic changes.
    if (pool != nullptr) {
        long nStates = pool->getNumberOfStates();
        for (long index = 0; index < nStates; index++) {
            double oldValue = allHeuristicValues[index];
            solver::StateInfo *info = pool->getInfoById(index);
            double newValue = heuristic(nullptr, info->getState(), nullptr);
            if (std::abs(newValue - oldValue) > 1e-5) {
                pool->setChangeFlags(info, solver::ChangeFlags::HEURISTIC);
            }
        }
    }
}



std::vector<std::unique_ptr<solver::State>> DronesModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs,
        long nParticles,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    DronesObservation const &observation =
            (static_cast<DronesObservation const &>(obs));
    ActionType actionType =
            (static_cast<DronesAction const &>(action).getActionType());
    // cout << "just to have a new line" << endl;
    // cout << "observation = " << observation << endl;
    // cout << "obs = " << obs << endl;

    while ((long)newParticles.size() < nParticles) {

	    SwarmVec OldSwarmPos;
		OldSwarmPos = observation.getSwarmVecPosition();
	    bool isLanded = false;
	    newParticles.push_back(std::make_unique<DronesState>(OldSwarmPos, isLanded));
  	}
    return newParticles;
}


/* --------------- Pretty printing methods ----------------- */
void DronesModel::dispCell(DronesCellType cellType, std::ostream &os) {
    switch (cellType) {
        case DronesCellType::EMPTY:
        os << " 0";
        break;
        case DronesCellType::WALL:
        os << "XX";
        break;
        default:
        os << "ER";
        break;
    }
}

void DronesModel::drawEnv(std::ostream &os) {
    for (std::vector<DronesCellType> &row : envMap_) {
        for (DronesCellType cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void DronesModel::drawSimulationState(solver::BeliefNode const *belief,
    solver::State const &state, std::ostream &os) {
    DronesState const &dronesState = static_cast<DronesState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    std::vector<std::vector<long>> particleCounts(nRows_,
        std::vector<long>(nCols_));
    SwarmVec swarmpos;
    swarmpos = dronesState.getSwarmVecPosition();
    std::vector<int> colors { 196, 161, 126, 91, 56, 21, 26, 31, 36, 41, 46 };
    if (options_->hasColorOutput) {
        os << "Color map: ";
        for (int color : colors) {
            os << "\033[38;5;" << color << "m";
            os << '*';
            os << "\033[0m";
        }
        os << endl;
    }
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            double proportion = (double) particleCounts[i][j]
            / particles.size();
            if (options_->hasColorOutput) {
                if (proportion > 0) {
                    int color = colors[proportion * (colors.size() - 1)];
                    os << "\033[38;5;" << color << "m";
                }
            }
            GridPosition pos(i, j);
            int swarmmember=0;
            for (unsigned int iter = 0; iter < swarmpos.size(); iter++)
            {
                if (pos == swarmpos.at(iter)){
                 swarmmember = iter+1;
             }
         }    

        if (swarmmember && envMap_[i][j] == DronesCellType::WALL) {
            os << "@";
        } else if (swarmmember) {
            os << swarmmember;
        } else {
            if (envMap_[i][j] == DronesCellType::WALL) {
                os << "X";
            } else {
                os << ".";
            }
        }
        if (options_->hasColorOutput) {
            os << "\033[0m";
        }
    }
    os << endl;
}
}


/* ---------------------- Basic customizations  ---------------------- */


// double DronesModel::getUpperBoundHeuristicValue(solver::State const &state) {
//     DronesState const &dronesState = static_cast<DronesState const &>(state);
//     cout << "this function " << endl;
//     double qVal = 80;

//     return qVal;
// }


/* ------- Customization of more complex solver functionality  --------- */
std::vector<std::unique_ptr<solver::DiscretizedPoint>> DronesModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < nActions_; code++) {
        allActions.push_back(std::make_unique<DronesAction>(code));
    }
    return allActions;
}

std::vector<std::vector<float>> DronesModel::getBeliefProportions(solver::BeliefNode const *belief) {
    std::vector<solver::State const *> particles = belief->getStates();
    std::vector<std::vector<long>> particleCounts(nRows_,  std::vector<long>(nCols_));
    std::vector<std::vector<float>> result;
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        result.push_back(std::vector<float>());
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            result[i].push_back((float) particleCounts[i][j]/particles.size());
        }
    }
    return result;
}

std::unique_ptr<solver::ActionPool> DronesModel::createActionPool(solver::Solver */*solver*/) {
return std::make_unique<solver::EnumeratedActionPool>(this, getAllActionsInOrder());
}
std::unique_ptr<solver::Serializer> DronesModel::createSerializer(solver::Solver *solver) {
    return std::make_unique<DronesTextSerializer>(solver);
}
} /* namespace drones */
