#include "action.h"

#include <boost/coroutine2/fixedsize_stack.hpp>
#include <ai/primitive/move_primitive.h>

Action::Action():
        primitive_sequence(
                boost::bind(&Action::runAction, this, _1)
        )
{
}

std::unique_ptr<Primitive> Action::updateStateAndGetNewPrimitive(World world) {
    curr_world = world;
    if (primitive_sequence){
        // TODO: COMMENTS HERE
        auto new_primitive = primitive_sequence.get();
        primitive_sequence();
        return new_primitive;
    }
    return std::unique_ptr<Primitive>{};
}

std::unique_ptr<Primitive> Action::runAction(primitive_coroutine::push_type& yield) {
    auto a = curr_world->ball();
    yield(
            std::make_unique<MovePrimitive>(10, Point(1,2), Angle::half(), 100)
            );
    yield(
            std::make_unique<MovePrimitive>(20, Point(3,4), Angle::half(), 100)
    );
    yield(
            std::make_unique<MovePrimitive>(30, Point(3,4), Angle::half(), 100)
    );
}

std::unique_ptr<Primitive>
Action::nullFunction(primitive_coroutine::push_type &yield) {
    return std::unique_ptr<Primitive>();
}

