#include <gtest/gtest.h>
#include <glog/logging.h>
#include "network.hpp"
#include "gridWorldPursuer.hpp"

namespace coopPE {



TEST(TestGridWorldPursuer, Vfn) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    std::shared_ptr<njm::tools::Rng> rng(new njm::tools::Rng);

    // fill uniformly
    std::vector<double> rewards(net->size(), 0.0);

    // gamma = 0.5
    rewards.at(0) = 1.0;
    const std::vector<double> vfn(GridWorldPursuer::calc_vfn(rewards, 0.5,
                    net));

    // one step
    EXPECT_NEAR(vfn.at(0), vfn.at(1), 1e-6);
    EXPECT_NEAR(vfn.at(0), vfn.at(3), 1e-6);
    EXPECT_NEAR(vfn.at(1), vfn.at(3), 1e-6);
    EXPECT_GT(vfn.at(0), vfn.at(2));
    EXPECT_GT(vfn.at(0), vfn.at(4));
    EXPECT_GT(vfn.at(0), vfn.at(6));
    EXPECT_GT(vfn.at(1), vfn.at(2));
    EXPECT_GT(vfn.at(1), vfn.at(4));
    EXPECT_GT(vfn.at(1), vfn.at(6));
    EXPECT_GT(vfn.at(3), vfn.at(2));
    EXPECT_GT(vfn.at(3), vfn.at(4));
    EXPECT_GT(vfn.at(3), vfn.at(6));

    // two steps
    EXPECT_NEAR(vfn.at(2), vfn.at(4), 1e-6);
    EXPECT_NEAR(vfn.at(2), vfn.at(6), 1e-6);
    EXPECT_NEAR(vfn.at(4), vfn.at(6), 1e-6);
    EXPECT_GT(vfn.at(2), vfn.at(5));
    EXPECT_GT(vfn.at(2), vfn.at(7));
    EXPECT_GT(vfn.at(4), vfn.at(5));
    EXPECT_GT(vfn.at(4), vfn.at(7));
    EXPECT_GT(vfn.at(6), vfn.at(5));
    EXPECT_GT(vfn.at(6), vfn.at(7));

    // three steps
    EXPECT_NEAR(vfn.at(5), vfn.at(7), 1e-6);
    EXPECT_GT(vfn.at(5), vfn.at(8));
    EXPECT_GT(vfn.at(7), vfn.at(8));


    // actual values
    EXPECT_NEAR(vfn.at(0), 2, 1e-6);
    EXPECT_NEAR(vfn.at(1), 2, 1e-6);
    EXPECT_NEAR(vfn.at(2), 1, 1e-6);
    EXPECT_NEAR(vfn.at(3), 2, 1e-6);
    EXPECT_NEAR(vfn.at(4), 1, 1e-6);
    EXPECT_NEAR(vfn.at(5), 0.5, 1e-6);
    EXPECT_NEAR(vfn.at(6), 1, 1e-6);
    EXPECT_NEAR(vfn.at(7), 0.5, 1e-6);
    EXPECT_NEAR(vfn.at(8), 0.25, 1e-6);
}





} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
