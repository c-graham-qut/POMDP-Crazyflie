/** @file DronesOptions.hpp
 *
 * Defines the DronesOptions class, which specifies the configuration settings available for the
 * Drones problem.
 */
#ifndef DRONESOPTIONS_HPP_
#define DRONESOPTIONS_HPP_

#include <string>                       // for string

#include "problems/shared/SharedOptions.hpp"

namespace drones {
/** A class defining the configuration settings for the Drones problem. */
struct DronesOptions : public shared::SharedOptions {
    DronesOptions() = default;
    virtual ~DronesOptions() = default;

    /* -------- Settings specific to the Drones POMDP -------- */
    /** Path to the map file (relative to SharedOptions::baseConfigPath) */
    std::string mapPath = "";
    double measurement_uncertainty = 0.0;
    double longitudinal_movement_uncertainty = 0.0;
    double initialPositions = 0.0;
    double lateral_movement_uncertainty = 0.0;
    double stepCost = 0.0;
    /** Path to vrep scene drones.ttt */
    std::string vrepScenePath = "";

    /** Constructs an OptionParser instance that will parse configuration settings for the Drones
     * problem into an instance of DronesOptions.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/drones");
        addDronesOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the Drones problem to the given parser. */
    static void addDronesOptions(options::OptionParser *parser) {
        parser->addOption<std::string>("problem", "mapPath", &DronesOptions::mapPath);
        parser->addValueArg<std::string>("problem", "mapPath", &DronesOptions::mapPath,
                "", "map", "the path to the map file (relative to the base config path)", "path");
        parser->addOption<double>("problem", "measurement_uncertainty", &DronesOptions::measurement_uncertainty);
        parser->addOption<double>("problem", "longitudinal_movement_uncertainty", &DronesOptions::longitudinal_movement_uncertainty);
        parser->addOption<double>("problem", "lateral_movement_uncertainty",
                &DronesOptions::lateral_movement_uncertainty);
        parser->addOption<double>("problem", "stepCost", &DronesOptions::stepCost);
        parser->addOptionWithDefault<std::string>("ros", "vrepScenePath",
                &DronesOptions::vrepScenePath, "");
    }
};
} /* namespace drones */

#endif /* DRONESOPTIONS_HPP_ */
