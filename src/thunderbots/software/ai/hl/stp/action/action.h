#pragma once

#include <ai/world/world.h>
#include <ai/primitive/primitive.h>
#include <boost/coroutine2/all.hpp>

// TODO: put this somewhere sane
typedef boost::coroutines2::coroutine< char > coro_t;

class Action {
public:

    // TODO: javadoc comment
    Action();

    // TODO: javadoc comment
    Primitive updateStateAndGetNewPrimitive(World world);

    //// TODO: javadoc comment
    //Primitive getCurrentPrimitive();

private:
    // TODO: javadoc comment
    void runAction();

    // TODO: javadoc comment
    std::optional<World> curr_world;

    // TODO: javadoc comment
    std::unique_ptr<Primitive> curr_primitive;

    coro_t::pull_type
};
