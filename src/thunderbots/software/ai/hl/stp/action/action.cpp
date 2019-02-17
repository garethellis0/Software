#include "action.h"

Action::Action() {
}

Primitive Action::updateStateAndGetNewPrimitive(World world) {
    curr_world = world;
}

