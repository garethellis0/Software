#include <gtest/gtest.h>
#include "ai/hl/stp/action/action.h"
#include "test/test_util/test_util.h"
#include <boost/coroutine2/all.hpp>

using namespace boost::coroutines2;


TEST(ActionTest, coroutine_test) {
    Action a;
    World w = ::Test::TestUtil::createBlankTestingWorld();
    std::unique_ptr<Primitive> p = a.updateStateAndGetNewPrimitive(w);
    p = a.updateStateAndGetNewPrimitive(w);
    p = a.updateStateAndGetNewPrimitive(w);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto val = RUN_ALL_TESTS();
    return val;
}

