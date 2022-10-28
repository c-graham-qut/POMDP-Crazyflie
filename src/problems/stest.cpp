/** @file drones/stest.cpp
 *
 * Defines the main method for the "stest" executable for the Drones POMDP, which tests the
 * serialization methods for Drones by deserializing and re-serializing a policy file.
 */
#include "problems/shared/stest.hpp"

#include "DronesModel.hpp"                 // for DronesModel
#include "DronesOptions.hpp"               // for DronesOptions

/** The main method for the "stest" executable for Drones. */
int main(int argc, char const *argv[]) {
    return stest<drones::DronesModel, drones::DronesOptions>(argc, argv);
}
