/** @file simulate.hpp
 *
 * Contains a generic function for running ABT simulations, which can be used to form the main
 * method of a problem-specific "simulate" executable.
 */
#ifndef SIMULATE_HPP_
#define SIMULATE_HPP_

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout
#include <map>
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator
#include <stdlib.h>                     // for sleep

#include "global.hpp"                     // for RandomGenerator, make_unique

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/ModelChange.hpp"       // for ModelChange
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for operator<<, State

#include "solver/serialization/Serializer.hpp"        // for Serializer

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Simulator.hpp"            // for Simulator
#include "solver/Solver.hpp"            // for Solver

#include "DronesAction.hpp"

#ifdef GOOGLE_PROFILER
#include <google/profiler.h>
#endif

using std::cout;
using std::endl;

/** A template method to run a simulation for the given model and options classes. */
template<typename ModelType, typename OptionsType>
int simulate_drones(int argc, char const *argv[]) {
    std::unique_ptr<options::OptionParser> parser = OptionsType::makeParser(true);

    OptionsType options;
    std::string workingDir = tapir::get_current_directory();
    try {
        parser->setOptions(&options);
        parser->parseCmdLine(argc, argv);
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(options.baseConfigPath);
        }
        if (!options.configPath.empty()) {
            parser->parseCfgFile(options.configPath);
        }
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(workingDir);
        }
        parser->finalize();
    } catch (options::OptionParsingException const &e) {
        std::cerr << e.what() << std::endl;
        return 2;
    }

    if (options.seed == 0) {
        options.seed = std::time(nullptr);
    }
    cout << "Global seed: " << options.seed << endl << endl;
    RandomGenerator randGen;
    randGen.seed(options.seed);
    randGen.discard(10);

    std::ofstream os(options.logPath);

    if (options.rngState > 0) {
        std::stringstream sstr;
        sstr << options.rngState;
        sstr >> randGen;
        cout << "Loaded PRNG state " << options.rngState << endl;
    }

    double totalReward = 0;
    double totalTime = 0;
    double totalNSteps = 0;

#ifdef GOOGLE_PROFILER
    ProfilerStart("simulate.prof");
#endif

    // for (long runNumber = 0; runNumber < options.nRuns; runNumber++) {
        long runNumber = 0;
        cout << "Run #" << runNumber+1 << endl;
        cout << "PRNG engine state: " << randGen << endl;

        // We want the simulated history to be independent of the solver's searching,
        // so we create a different random generator here.
        RandomGenerator solverGen(randGen);
        // Advance it forward a long way to avoid correlation between the solver and simulator.
        solverGen.discard(10000);

        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(options.baseConfigPath);
        }
        std::unique_ptr<ModelType> solverModel = std::make_unique<ModelType>(&solverGen,
                std::make_unique<OptionsType>(options));;
        solver::Solver solver(std::move(solverModel));

        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(workingDir);
        }
        if (options.loadInitialPolicy) {
            cout << "Loading policy... " << endl;
            std::ifstream inFile;
            inFile.open(options.policyPath);
            if (!inFile.is_open()) {
                std::ostringstream message;
                message << "Failed to open " << options.policyPath;
                debug::show_message(message.str());
                return 1;
            }
            solver.getSerializer()->load(inFile);
            inFile.close();
        } else {
            cout << "Starting from empty policy. " << endl;
            solver.initializeEmpty();
        }
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(options.baseConfigPath);
        }

        std::unique_ptr<ModelType> simulatorModel = std::make_unique<ModelType>(&randGen,
                std::make_unique<OptionsType>(options));
        solver::Simulator simulator(std::move(simulatorModel), &solver, options.areDynamic);
        if (options.hasChanges) {
            simulator.loadChangeSequence(options.changesPath);
        }
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(workingDir);
        }

        simulator.setMaxStepCount(options.nSimulationSteps);
        cout << "Running..." << endl;

        double tStart = tapir::clock_ms();
        solver::HistoryEntry::IdType entryNo = 0;
        std::ifstream inputfile;
        std::ofstream outputfile;
        double reward = 0;
        bool running = true;
        while (running){
            std::tie(reward, running) = simulator.HiLstepSimulation(entryNo);
            // changes directory to open changes file
            if (!options.baseConfigPath.empty()) {
                tapir::change_directory(options.baseConfigPath);
            }
            // opens changes file, reads it and closes it
            if (options.hasChanges) {
                simulator.loadChangeSequence(options.changesPath);
            }
            // changes directory back to normal working directo./siry
            if (!options.baseConfigPath.empty()) {
                tapir::change_directory(workingDir);
            }
            // reward += step_reward;
            // cout << "my reward = " << reward << endl;
            entryNo++;
        }


        double totT = tapir::clock_ms() - tStart;
        long actualNSteps = simulator.getStepCount();
        totalReward += reward;
        totalTime += totT;
        totalNSteps += actualNSteps;

        solver::HistorySequence *sequence = simulator.getHistory();

        cout << "\n\nFinal State: " << *sequence->getLastEntry()->getState();
        cout << endl;

        cout << "Total discounted reward: " << reward << "\n" << endl;
        cout << "# of steps: " << actualNSteps << endl;
        cout << "Time spent on changes: ";
        cout << simulator.getTotalChangingTime() << "ms" << endl;
        cout << "Time spent on policy updates: ";
        cout << simulator.getTotalImprovementTime() << "ms" << endl;
        cout << "Time spent replenishing particles: ";
        cout << simulator.getTotalReplenishingTime() << "ms" << endl;
        cout << "Time spent pruning: ";
        cout << simulator.getTotalPruningTime() << "ms" << endl;
        cout << "Total time taken: " << totT << "ms" << endl;
        // if (options.savePolicy) {
        //     // Write the final policy to a file.
        //     cout << "Saving final policy..." << endl;
        //     std::ofstream outFile;
        //     std::ostringstream sstr;
        //     sstr << "final-" << runNumber << ".pol";
        //     outFile.open(sstr.str());
        //     solver.getSerializer()->save(outFile);
        //     outFile.close();
        //     cout << "Finished saving." << endl;
        // }
        // cout << "Run complete!" << endl << endl;
    // }

#ifdef GOOGLE_PROFILER
    ProfilerStop();
#endif

    os.close();

    cout << options.nRuns << " runs completed." << endl;
    cout << "Mean reward: " << totalReward / options.nRuns << endl;
    cout << "Mean number of steps: " << totalNSteps / options.nRuns << endl;
    cout << "Mean time taken: " << totalTime / options.nRuns << "ms" << endl;
    cout << "Mean time per step: " << totalTime / totalNSteps << "ms" << endl;
    return 0;
}

#endif /* SIMULATE_DRONES_HPP_ */

