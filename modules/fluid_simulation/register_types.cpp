
/* register_types.cpp */

#include "register_types.h"

#include "core/object/class_db.h"
#include "fluid_simulation.h"

void initialize_fluid_simulation_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
	}
	ClassDB::register_class<FluidSimulator>();
}

void uninitialize_fluid_simulation_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
	}
   // Nothing to do here in this example.
}
