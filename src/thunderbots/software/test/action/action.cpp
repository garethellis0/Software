#include "ai/hl/stp/action/move.h"
#include <gtest/gtest.h>

using namespace boost::coroutines2;

void printResults(std::optional<int> r) {
    if(r) {
        std::cout << *r << std::endl;
    }else {
        std::cout << "action done" << std::endl;
    }
}

TEST(ActionTest, coroutine_test)
{
    ActionMove action_move = ActionMove(1);
    for(int i = 0; i < 5; i++) {
        auto p = action_move.getPrimitive();
        printResults(p);
    }

//    action_move.updateState(2);
//
//    for(int i = 0; i < 5; i++) {
//        auto p = action_move.getPrimitive();
//        printResults(p);
//    }

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
