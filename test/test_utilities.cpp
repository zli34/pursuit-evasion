#include <gtest/gtest.h>
#include <glog/logging.h>
#include "utilities.hpp"
#include "network.hpp"

namespace coopPE {



TEST(TestUtilities, NextMoveOneAction) {
    std::vector<proto::Unit_Action> actions(1, proto::Unit_Action_NOTHING);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_UP);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_DOWN);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_LEFT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_RIGHT);

    EXPECT_FALSE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_NOTHING);
}


TEST(TestUtilities, NextMoveTwoActions) {
    std::vector<proto::Unit_Action> actions(2, proto::Unit_Action_NOTHING);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_UP);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_NOTHING);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_DOWN);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_NOTHING);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_LEFT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_NOTHING);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_RIGHT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_NOTHING);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_NOTHING);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_UP);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_UP);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_UP);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_DOWN);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_UP);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_LEFT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_UP);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_RIGHT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_UP);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_NOTHING);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_DOWN);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_UP);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_DOWN);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_DOWN);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_DOWN);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_LEFT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_DOWN);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_RIGHT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_DOWN);


    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_NOTHING);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_LEFT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_UP);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_LEFT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_DOWN);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_LEFT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_LEFT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_LEFT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_RIGHT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_LEFT);


    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_NOTHING);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_RIGHT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_UP);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_RIGHT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_DOWN);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_RIGHT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_LEFT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_RIGHT);

    EXPECT_TRUE(next_move(actions));
    EXPECT_EQ(actions.at(0), proto::Unit_Action_RIGHT);
    EXPECT_EQ(actions.at(1), proto::Unit_Action_RIGHT);

    EXPECT_FALSE(next_move(actions));
}


TEST(TestUtilities, FindPeaks) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    const std::vector<uint32_t> peaks(find_peaks({/*0*/1.0, /*1*/1.0, /*2*/0.0,
                                                  /*3*/1.0, /*4*/0.7, /*5*/0.1,
                                                  /*6*/0.0, /*7*/0.1, /*8*/0.0},
                    2, net));

    EXPECT_EQ(peaks.at(0), 0);
    EXPECT_EQ(peaks.at(1), 4);

}


TEST(TestUtilities, AssignPeaks) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    const std::vector<double> probs({
                1.0, 1.0, 0.0,
                1.0, 0.7, 0.1,
                0.0, 0.1, 0.0});

    const std::vector<uint32_t> peaks(find_peaks(probs, 2, net));

    EXPECT_EQ(peaks.at(0), 0);
    EXPECT_EQ(peaks.at(1), 4);

    {
        const std::vector<uint32_t> assignments(assign_peaks({1, 7},
                        peaks, net));

        EXPECT_EQ(assignments.at(0), 0);
        EXPECT_EQ(assignments.at(1), 1);
    }

    {
        const std::vector<uint32_t> assignments(assign_peaks({7, 1},
                        peaks, net));

        EXPECT_EQ(assignments.at(0), 1);
        EXPECT_EQ(assignments.at(1), 0);
    }
}




} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
