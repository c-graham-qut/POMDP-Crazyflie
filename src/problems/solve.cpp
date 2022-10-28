/** @file problems/drones/solve.cpp
 *
 * Defines the main method for the "solve" executable for the Drones POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "DronesModel.hpp"                 // for DronesModel
#include "DronesOptions.hpp"               // for DronesOptions

/** The main method for the "solve" executable for Drones. */
int main(int argc, char const *argv[]) {
    return solve<drones::DronesModel, drones::DronesOptions>(argc, argv);
}
