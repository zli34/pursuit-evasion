#include <gtest/gtest.h>
#include <glog/logging.h>
#include "network.hpp"
#include "rwPursuer.hpp"

namespace coopPE {



TEST(TestRwPursuer, Moving) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    std::map<uint32_t, InformantTip> tip_history;

    // 0.0 drift
    const uint32_t num_units = 3;
    RwPursuer pursuer(num_units, net, {{0}, {1}, {2}});
    pursuer.reset();

    std::vector<proto::Unit> units(num_units);
    EXPECT_EQ(pursuer.starting_locs().size(), num_units);
    for (uint32_t i = 0; i < num_units; ++i) {
        units.at(i).set_loc(pursuer.starting_locs().at(i));
    }
    EXPECT_EQ(units.at(0).loc(), 0);
    EXPECT_EQ(units.at(1).loc(), 1);
    EXPECT_EQ(units.at(2).loc(), 2);

    std::vector<std::map<proto::Unit_Action, double> > freq(3);
    for (uint32_t i = 0; i < num_units; ++i) {
        freq.at(i)[proto::Unit_Action_LEFT] = 0.0;
        freq.at(i)[proto::Unit_Action_RIGHT] = 0.0;
        freq.at(i)[proto::Unit_Action_DOWN] = 0.0;
        freq.at(i)[proto::Unit_Action_UP] = 0.0;
        freq.at(i)[proto::Unit_Action_NOTHING] = 0.0;
    }

    const uint32_t num_reps(10000);
    for (uint32_t i = 0; i < num_reps; ++i) {
        const std::vector<proto::Unit_Action> moves(pursuer.decide_moves(units, {},
                        tip_history));
        for (uint32_t j = 0; j < num_units; ++j) {
            freq.at(j).at(moves.at(j)) += 1.0 / num_reps;
        }
    }

    for (uint32_t i = 0; i < num_units; ++i) {
        EXPECT_NEAR(freq.at(i)[proto::Unit_Action_LEFT], 0.2, 1e-2);
        EXPECT_NEAR(freq.at(i)[proto::Unit_Action_RIGHT], 0.2, 1e-2);
        EXPECT_NEAR(freq.at(i)[proto::Unit_Action_UP], 0.2, 1e-2);
        EXPECT_NEAR(freq.at(i)[proto::Unit_Action_DOWN], 0.2, 1e-2);
        EXPECT_NEAR(freq.at(i)[proto::Unit_Action_NOTHING], 0.2, 1e-2);
    }
}





} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
