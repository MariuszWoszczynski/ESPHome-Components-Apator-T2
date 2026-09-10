#include "meters.h"

// meters.cc keeps the driver registry behind lazily allocated global pointers.
// Some legacy registration paths reach allDrivers() from static constructors
// before those pointers have been initialized.  Do not rely on assert() side
// effects for initialization: assertions may be configured differently by the
// ESP-IDF/ESPHome build.
void verifyDriverLookupCreated();

namespace {

// C++ static driver registrations use the default init priority.  Initialize
// the registry first so allDrivers() is safe during those registrations.
__attribute__((constructor(101))) void initialize_wmbus_driver_registry() { verifyDriverLookupCreated(); }

}  // namespace
