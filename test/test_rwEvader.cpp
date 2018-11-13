#include <gtest/gtest.h>
#include <glog/logging.h>
#include "network.hpp"
#include "rwEvader.hpp"

namespace coopPE {





TEST(TestRwEvader, Moving) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));


    // 1.0 drift
    {
        RwEvader evader(net, {3}, {4}, {1.0});
        evader.reset();

        proto::Unit unit;
        unit.set_loc(evader.starting_loc());
        EXPECT_EQ(unit.loc(), 3);

        EXPECT_EQ(evader.decide_move(unit, {}), proto::Unit_Action_DOWN);
    }

    // 0.5 drift
    {
        RwEvader evader(net, {3}, {4}, {0.5});
        evader.reset();

        proto::Unit unit;
        unit.set_loc(evader.starting_loc());
        EXPECT_EQ(unit.loc(), 3);

        std::map<proto::Unit_Action, double> freq;
        freq[proto::Unit_Action_LEFT] = 0.0;
        freq[proto::Unit_Action_RIGHT] = 0.0;
        freq[proto::Unit_Action_DOWN] = 0.0;
        freq[proto::Unit_Action_UP] = 0.0;
        freq[proto::Unit_Action_NOTHING] = 0.0;

        const uint32_t num_reps(10000);
        for (uint32_t i = 0; i < num_reps; ++i) {
            freq.at(evader.decide_move(unit, {})) += 1.0 / num_reps;
        }

        EXPECT_NEAR(freq[proto::Unit_Action_LEFT], 0.5 * 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_RIGHT], 0.5 * 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_UP], 0.5 * 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_DOWN], 0.5 * 0.2 + 0.5, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_NOTHING], 0.5 * 0.2, 1e-2);
    }

    // 0.0 drift
    {
        RwEvader evader(net, {3}, {4}, {0.0});
        evader.reset();

        proto::Unit unit;
        unit.set_loc(evader.starting_loc());
        EXPECT_EQ(unit.loc(), 3);

        std::map<proto::Unit_Action, double> freq;
        freq[proto::Unit_Action_LEFT] = 0.0;
        freq[proto::Unit_Action_RIGHT] = 0.0;
        freq[proto::Unit_Action_DOWN] = 0.0;
        freq[proto::Unit_Action_UP] = 0.0;
        freq[proto::Unit_Action_NOTHING] = 0.0;

        const uint32_t num_reps(10000);
        for (uint32_t i = 0; i < num_reps; ++i) {
            freq.at(evader.decide_move(unit, {})) += 1.0 / num_reps;
        }

        EXPECT_NEAR(freq[proto::Unit_Action_LEFT], 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_RIGHT], 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_UP], 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_DOWN], 0.2, 1e-2);
        EXPECT_NEAR(freq[proto::Unit_Action_NOTHING], 0.2, 1e-2);
    }
}





} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
