/** @file problems/drones/simulate.cpp
 *
 * Defines the main method for the "simulate" executable for the Drones POMDP, which runs online
 * simulations to test the performance of the solver.
 */
#include "problems/shared/simulate.hpp"

#include "DronesModel.hpp"                 // for DronesModel
#include "DronesOptions.hpp"               // for DronesOptions

/** The main method for the "simulate" executable for Drones. */
int main(int argc, char const *argv[]) {
    return simulate<drones::DronesModel, drones::DronesOptions>(argc, argv);
}
