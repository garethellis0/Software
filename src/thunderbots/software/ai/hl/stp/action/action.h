#pragma once

#include <ai/world/world.h>
#include <ai/primitive/primitive.h>
#include <boost/coroutine2/all.hpp>

// TODO: put this somewhere sane
typedef boost::coroutines2::coroutine<std::unique_ptr<Primitive>> primitive_coroutine;

class Action {
public:

    // TODO: javadoc comment
    Action();

    // TODO: javadoc comment
    std::unique_ptr<Primitive> updateStateAndGetNewPrimitive(World world);

    //// TODO: javadoc comment
    //Primitive getCurrentPrimitive();

private:
    // TODO: javadoc comment
    std::unique_ptr<Primitive> runAction(primitive_coroutine::push_type& yield);

    static std::unique_ptr<Primitive> nullFunction(primitive_coroutine::push_type& yield);

    // TODO: javadoc comment
    std::optional<World> curr_world;

    // TODO: javadoc comment
    std::unique_ptr<Primitive> curr_primitive;

    primitive_coroutine::pull_type primitive_sequence;
};
