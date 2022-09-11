/** @file DronesModel.hpp
 *
 * Contains DronesModel, which implements the core Model interface for the Drones POMDP.
 */
#ifndef DRONESMODEL_HPP_
#define DRONESMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "global.hpp"                     // for RandomGenerator

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/ModelChange.hpp"             // for ModelChange
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"

#include "DronesAction.hpp"
#include "DronesOptions.hpp"
// #include "DronesMdpSolver.hpp"

namespace solver {
class StatePool;
} /* namespace solver */

/** A namespace to hold the various classes used for the Drones POMDP model. */
namespace drones {
class DronesObervation;
class DronesState;

/** Represents a change in the Drones model. */
struct DronesChange : public solver::ModelChange {
    /** The change type for this change - this should be one of:
     * - "Add Obstacles" - to add new obstacles to the Drones problem.
     * - "Remove Obstacles" - to remove existing obstacles from the Drones problem.
     */
    std::string changeType = "";
    /** The first row number where the change applies. */
    long i0 = 0;
    /** The last row number where the change applies. */
    long i1 = 0;
    /** The first column number where the change applies. */
    long j0 = 0;
    /** The last column number where the change applies. */
    long j1 = 0;
};

/** A parser for a simple upper bound heuristic for Drones.
 *
 * The actual function is defined in DronesModel::getUpperBoundHeuristicValue; this parser allows
 * that heuristic to be selected by using the string "upper()" in the configuration file.
 */
// class DronesUBParser : public shared::Parser<solver::HeuristicFunction> {
// public:
//     /** Creates a new DronesUBParser associated with the given DronesModel instance. */
//     DronesUBParser(DronesModel *model);
//     virtual ~DronesUBParser() = default;
//     _NO_COPY_OR_MOVE(DronesUBParser);

//     virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args);

// private:
//     /** The DronesModel instance this heuristic parser is associated with. */
//     DronesModel *model_;
// };

/** The implementation of the Model interface for the Drones POMDP.
 *
 * See this paper http://www.cs.cmu.edu/~ggordon/jpineau-ggordon-thrun.ijcai03.pdf
 * for a description of the Drones problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class DronesModel: public shared::ModelWithProgramOptions {
    friend class DronesObservation;
    // friend class DronesMdpSolver;

  public:
    /** Constructs a new DronesModel instance with the given random number engine, and the given set
     * of configuration options.
     */
    DronesModel(RandomGenerator *randGen, std::unique_ptr<DronesOptions> options);

    ~DronesModel() = default;
    _NO_COPY_OR_MOVE(DronesModel);

    /** The cells are either empty or walls. */
    enum class DronesCellType : int {
        /** An empty cell. */
        EMPTY = 0,
        /* A wall. */
        WALL = -1
    };

    /******************** Added by Josh **************************/

    /** Get 2D vector representing the current environment map */
    inline const std::vector<std::vector<DronesCellType>>& getEnvMap() {
        return envMap_;
    }

    /**
     * Returns proportion of belief particles about the target's
     * position for each grid position in the map
     */
    std::vector<std::vector<float>> getBeliefProportions(solver::BeliefNode const *belief);

    /************************************************************/

    /** Returns the resulting coordinates of an agent after it takes the given action type from the
     * given position.
     *
     * The boolean flag will be false if the agent's move represents an attempt to move into an
     * obstacle or off the edge of the map, which in the Drones POMDP simply causes them to stay
     * in the same position.
     *
     * This flag is mostly not used as there is no penalty for this in Drones, and the returned
     * position already reflects them staying still.
     */
    // std::pair<GridPosition, bool> getMovedPos(GridPosition const &position, ActionType action);

    /** Generates a new opponent position based on the current positions of the robot and the
     * opponent.
     */

    std::pair<SwarmVec, bool>  getMovedSwarmPos(SwarmVec const &swarmPos, 
        ActionType action);
    //  new function for swarms

    /* ---------- Custom getters for extra functionality  ---------- */
    /** Returns the number of rows in the map for this DronesModel instance. */
    long getNRows() const {
        return nRows_;
    }
    /** Returns the number of columns in the map for this DronesModel instance. */
    long getNCols() const {
        return nCols_;
    }
    /* --------------- The model interface proper ----------------- */
    std::unique_ptr<solver::State> sampleAnInitState() override;
    std::unique_ptr<solver::State> sampleStateUninformed() override;
    bool isTerminal(solver::State const &state) override;
    bool isValid(solver::State const &state) override;

    /* -------------------- Black box dynamics ---------------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/) override;
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::State const */*state*/,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/,
            solver::State const &nextState) override;
    virtual double generateReward(
                solver::State const &state,
                solver::Action const &action,
                solver::TransitionParameters const */*tp*/,
                solver::State const &nextState) override;
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;
    virtual Model::StepResult generateRealStep(solver::State const &state,
            solver::Action const &action) override;

    /* -------------- Methods for handling model changes ---------------- */
    virtual void applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
             solver::Solver *solver, bool print) override;


    /* ------------ Methods for handling particle depletion -------------- */
    /** Generates particles for Drones using a particle filter from the previous belief.*/


    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles) override;



    /* --------------- Pretty printing methods ----------------- */
    /** Prints a single cell of the map out to the given output stream. */
    virtual void dispCell(DronesCellType cellType, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;


    /* ---------------------- Basic customizations  ---------------------- */
    // virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
    //             solver::State const *state, solver::HistoricalData const *data) override;

    /** Returns an upper bound heuristic value for the given state.
     *
     * This upper bound assumes that the opponent will not move, and hence the heuristic value
     * simply calculates the discounted total reward, including the cost of moving one square at a
     * time until the robot reaches the opponent's current square, and then the reward for dronesging
     * the opponent at that time.
     */
    // virtual double getUpperBoundHeuristicValue(solver::State const &state);

    /* ------- Customization of more complex solver functionality  --------- */
    /** Returns all of the actions available for the Drones POMDP, in the order of their enumeration
     * (as specified by drones::ActionType).
     */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

  private:
    /** Calculates the distances from the given position to all other parts of the map. */
    // void calculateDistancesFrom(GridPosition position);
    /** Calculates all pairwise distances on the map. */
    // void calculatePairwiseDistances();

    /** Initialises the required data structures and variables for this model. */
    void initialize();

    /** Generates a random empty grid cell. */
    GridPosition randomEmptyCell();

    /** Generates a next state for the given state and action, as well as a boolean flag that will
     * be true if the action moved into a wall, and false otherwise.
     *
     * Moving into a wall in Drones simply means nothing happens - there is no associated penalty;
     * as such, this flag is mostly not used in the Drones problem.
     */
    std::pair<std::unique_ptr<DronesState>, bool> makeNextState(
            solver::State const &state, solver::Action const &action);

    /** Generates an observation given the resulting next state, after the Drones robot has made its
     * action.
     */
    std::unique_ptr<solver::Observation> makeObservation(DronesState const &nextState);

    /** Returns true iff the given GridPosition represents a valid square that an agent could be
     * in - that is, the square must be empty, and within the bounds of the map.
     */
    bool isValid(GridPosition const &pos);

    /** The DronesOptions instance associated with this model. */
    DronesOptions *options_;

    /** The uncertainty in the measurement of the position of the UAVs. */
    double measurement_uncertainty_;
    /** The uncertainty of the forward movement of the UAVs. */
    double longitudinal_movement_uncertainty_;
    /** The probability that the opponent will stay still. */
    double lateral_movement_uncertainty_;
    /** The cost for every step. */
    double stepCost_;

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;

    SwarmVec initialPositions_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<DronesCellType>> envMap_;

    /** The number of possible actions in the Drones POMDP. */
    long nActions_;
};
} /* namespace drones */

#endif /* DRONESMODEL_HPP_ */
