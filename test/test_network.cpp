#include <gtest/gtest.h>
#include <glog/logging.h>
#include "network.hpp"

namespace coopPE {





TEST(TestNetwork,TestGridNetworkNoWrapAdjacency) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    std::shared_ptr<Network> net = Network::gen_network(init);

    ASSERT_EQ(net->size(),9);

    // node 0
    EXPECT_EQ(net->node(0).index(), 0);
    EXPECT_NEAR(net->node(0).x(), 0., 1e-6);
    EXPECT_NEAR(net->node(0).y(), 0., 1e-6);
    EXPECT_EQ(net->node(0).up(), 0);
    EXPECT_EQ(net->node(0).down(), 1);
    EXPECT_EQ(net->node(0).left(), 0);
    EXPECT_EQ(net->node(0).right(), 3);

    // node 1
    EXPECT_EQ(net->node(1).index(), 1);
    EXPECT_NEAR(net->node(1).x(), 0., 1e-6);
    EXPECT_NEAR(net->node(1).y(), 0.5, 1e-6);
    EXPECT_EQ(net->node(1).up(), 0);
    EXPECT_EQ(net->node(1).down(), 2);
    EXPECT_EQ(net->node(1).left(), 1);
    EXPECT_EQ(net->node(1).right(), 4);

    // node 2
    EXPECT_EQ(net->node(2).index(), 2);
    EXPECT_NEAR(net->node(2).x(), 0., 1e-6);
    EXPECT_NEAR(net->node(2).y(), 1., 1e-6);
    EXPECT_EQ(net->node(2).up(), 1);
    EXPECT_EQ(net->node(2).down(), 2);
    EXPECT_EQ(net->node(2).left(), 2);
    EXPECT_EQ(net->node(2).right(), 5);

    // node 3
    EXPECT_EQ(net->node(3).index(), 3);
    EXPECT_NEAR(net->node(3).x(), 0.5, 1e-6);
    EXPECT_NEAR(net->node(3).y(), 0., 1e-6);
    EXPECT_EQ(net->node(3).up(), 3);
    EXPECT_EQ(net->node(3).down(), 4);
    EXPECT_EQ(net->node(3).left(), 0);
    EXPECT_EQ(net->node(3).right(), 6);

    // node 4
    EXPECT_EQ(net->node(4).index(), 4);
    EXPECT_NEAR(net->node(4).x(), 0.5, 1e-6);
    EXPECT_NEAR(net->node(4).y(), 0.5, 1e-6);
    EXPECT_EQ(net->node(4).up(), 3);
    EXPECT_EQ(net->node(4).down(), 5);
    EXPECT_EQ(net->node(4).left(), 1);
    EXPECT_EQ(net->node(4).right(), 7);

    // node 5
    EXPECT_EQ(net->node(5).index(), 5);
    EXPECT_NEAR(net->node(5).x(), 0.5, 1e-6);
    EXPECT_NEAR(net->node(5).y(), 1., 1e-6);
    EXPECT_EQ(net->node(5).up(), 4);
    EXPECT_EQ(net->node(5).down(), 5);
    EXPECT_EQ(net->node(5).left(), 2);
    EXPECT_EQ(net->node(5).right(), 8);

    // node 6
    EXPECT_EQ(net->node(6).index(), 6);
    EXPECT_NEAR(net->node(6).x(), 1., 1e-6);
    EXPECT_NEAR(net->node(6).y(), 0., 1e-6);
    EXPECT_EQ(net->node(6).up(), 6);
    EXPECT_EQ(net->node(6).down(), 7);
    EXPECT_EQ(net->node(6).left(), 3);
    EXPECT_EQ(net->node(6).right(), 6);

    // node 7
    EXPECT_EQ(net->node(7).index(), 7);
    EXPECT_NEAR(net->node(7).x(), 1., 1e-6);
    EXPECT_NEAR(net->node(7).y(), 0.5, 1e-6);
    EXPECT_EQ(net->node(7).up(), 6);
    EXPECT_EQ(net->node(7).down(), 8);
    EXPECT_EQ(net->node(7).left(), 4);
    EXPECT_EQ(net->node(7).right(), 7);

    // node 8
    EXPECT_EQ(net->node(8).index(), 8);
    EXPECT_NEAR(net->node(8).x(), 1., 1e-6);
    EXPECT_NEAR(net->node(8).y(), 1., 1e-6);
    EXPECT_EQ(net->node(8).up(), 7);
    EXPECT_EQ(net->node(8).down(), 8);
    EXPECT_EQ(net->node(8).left(), 5);
    EXPECT_EQ(net->node(8).right(), 8);
}



TEST(TestNetwork,TestGridNetworkWrapAdjacency) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(true);
    init.set_type(proto::NetworkInit_NetType_GRID);

    std::shared_ptr<Network> net = Network::gen_network(init);

    ASSERT_EQ(net->size(),9);

    // node 0
    EXPECT_EQ(net->node(0).index(), 0);
    EXPECT_NEAR(net->node(0).x(), 0., 1e-6);
    EXPECT_NEAR(net->node(0).y(), 0., 1e-6);
    EXPECT_EQ(net->node(0).up(), 2);
    EXPECT_EQ(net->node(0).down(), 1);
    EXPECT_EQ(net->node(0).left(), 6);
    EXPECT_EQ(net->node(0).right(), 3);

    // node 1
    EXPECT_EQ(net->node(1).index(), 1);
    EXPECT_NEAR(net->node(1).x(), 0., 1e-6);
    EXPECT_NEAR(net->node(1).y(), 0.5, 1e-6);
    EXPECT_EQ(net->node(1).up(), 0);
    EXPECT_EQ(net->node(1).down(), 2);
    EXPECT_EQ(net->node(1).left(), 7);
    EXPECT_EQ(net->node(1).right(), 4);

    // node 2
    EXPECT_EQ(net->node(2).index(), 2);
    EXPECT_NEAR(net->node(2).x(), 0., 1e-6);
    EXPECT_NEAR(net->node(2).y(), 1., 1e-6);
    EXPECT_EQ(net->node(2).up(), 1);
    EXPECT_EQ(net->node(2).down(), 0);
    EXPECT_EQ(net->node(2).left(), 8);
    EXPECT_EQ(net->node(2).right(), 5);

    // node 3
    EXPECT_EQ(net->node(3).index(), 3);
    EXPECT_NEAR(net->node(3).x(), 0.5, 1e-6);
    EXPECT_NEAR(net->node(3).y(), 0., 1e-6);
    EXPECT_EQ(net->node(3).up(), 5);
    EXPECT_EQ(net->node(3).down(), 4);
    EXPECT_EQ(net->node(3).left(), 0);
    EXPECT_EQ(net->node(3).right(), 6);

    // node 4
    EXPECT_EQ(net->node(4).index(), 4);
    EXPECT_NEAR(net->node(4).x(), 0.5, 1e-6);
    EXPECT_NEAR(net->node(4).y(), 0.5, 1e-6);
    EXPECT_EQ(net->node(4).up(), 3);
    EXPECT_EQ(net->node(4).down(), 5);
    EXPECT_EQ(net->node(4).left(), 1);
    EXPECT_EQ(net->node(4).right(), 7);

    // node 5
    EXPECT_EQ(net->node(5).index(), 5);
    EXPECT_NEAR(net->node(5).x(), 0.5, 1e-6);
    EXPECT_NEAR(net->node(5).y(), 1., 1e-6);
    EXPECT_EQ(net->node(5).up(), 4);
    EXPECT_EQ(net->node(5).down(), 3);
    EXPECT_EQ(net->node(5).left(), 2);
    EXPECT_EQ(net->node(5).right(), 8);

    // node 6
    EXPECT_EQ(net->node(6).index(), 6);
    EXPECT_NEAR(net->node(6).x(), 1., 1e-6);
    EXPECT_NEAR(net->node(6).y(), 0., 1e-6);
    EXPECT_EQ(net->node(6).up(), 8);
    EXPECT_EQ(net->node(6).down(), 7);
    EXPECT_EQ(net->node(6).left(), 3);
    EXPECT_EQ(net->node(6).right(), 0);

    // node 7
    EXPECT_EQ(net->node(7).index(), 7);
    EXPECT_NEAR(net->node(7).x(), 1., 1e-6);
    EXPECT_NEAR(net->node(7).y(), 0.5, 1e-6);
    EXPECT_EQ(net->node(7).up(), 6);
    EXPECT_EQ(net->node(7).down(), 8);
    EXPECT_EQ(net->node(7).left(), 4);
    EXPECT_EQ(net->node(7).right(), 1);

    // node 8
    EXPECT_EQ(net->node(8).index(), 8);
    EXPECT_NEAR(net->node(8).x(), 1., 1e-6);
    EXPECT_NEAR(net->node(8).y(), 1., 1e-6);
    EXPECT_EQ(net->node(8).up(), 7);
    EXPECT_EQ(net->node(8).down(), 6);
    EXPECT_EQ(net->node(8).left(), 5);
    EXPECT_EQ(net->node(8).right(), 2);
}


TEST(TestNetwork,TestGridNetworkNoWrapDistance) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    std::shared_ptr<Network> net = Network::gen_network(init);

    // node 0
    EXPECT_EQ(net->dist(0, 0), 0);
    EXPECT_EQ(net->dist(0, 1), 1);
    EXPECT_EQ(net->dist(0, 2), 2);
    EXPECT_EQ(net->dist(0, 3), 1);
    EXPECT_EQ(net->dist(0, 4), 2);
    EXPECT_EQ(net->dist(0, 5), 3);
    EXPECT_EQ(net->dist(0, 6), 2);
    EXPECT_EQ(net->dist(0, 7), 3);
    EXPECT_EQ(net->dist(0, 8), 4);

    // node 1
    EXPECT_EQ(net->dist(1, 0), 1);
    EXPECT_EQ(net->dist(1, 1), 0);
    EXPECT_EQ(net->dist(1, 2), 1);
    EXPECT_EQ(net->dist(1, 3), 2);
    EXPECT_EQ(net->dist(1, 4), 1);
    EXPECT_EQ(net->dist(1, 5), 2);
    EXPECT_EQ(net->dist(1, 6), 3);
    EXPECT_EQ(net->dist(1, 7), 2);
    EXPECT_EQ(net->dist(1, 8), 3);

    // node 2
    EXPECT_EQ(net->dist(2, 0), 2);
    EXPECT_EQ(net->dist(2, 1), 1);
    EXPECT_EQ(net->dist(2, 2), 0);
    EXPECT_EQ(net->dist(2, 3), 3);
    EXPECT_EQ(net->dist(2, 4), 2);
    EXPECT_EQ(net->dist(2, 5), 1);
    EXPECT_EQ(net->dist(2, 6), 4);
    EXPECT_EQ(net->dist(2, 7), 3);
    EXPECT_EQ(net->dist(2, 8), 2);

    // node 3
    EXPECT_EQ(net->dist(3, 0), 1);
    EXPECT_EQ(net->dist(3, 1), 2);
    EXPECT_EQ(net->dist(3, 2), 3);
    EXPECT_EQ(net->dist(3, 3), 0);
    EXPECT_EQ(net->dist(3, 4), 1);
    EXPECT_EQ(net->dist(3, 5), 2);
    EXPECT_EQ(net->dist(3, 6), 1);
    EXPECT_EQ(net->dist(3, 7), 2);
    EXPECT_EQ(net->dist(3, 8), 3);

    // node 4
    EXPECT_EQ(net->dist(4, 0), 2);
    EXPECT_EQ(net->dist(4, 1), 1);
    EXPECT_EQ(net->dist(4, 2), 2);
    EXPECT_EQ(net->dist(4, 3), 1);
    EXPECT_EQ(net->dist(4, 4), 0);
    EXPECT_EQ(net->dist(4, 5), 1);
    EXPECT_EQ(net->dist(4, 6), 2);
    EXPECT_EQ(net->dist(4, 7), 1);
    EXPECT_EQ(net->dist(4, 8), 2);

    // node 5
    EXPECT_EQ(net->dist(5, 0), 3);
    EXPECT_EQ(net->dist(5, 1), 2);
    EXPECT_EQ(net->dist(5, 2), 1);
    EXPECT_EQ(net->dist(5, 3), 2);
    EXPECT_EQ(net->dist(5, 4), 1);
    EXPECT_EQ(net->dist(5, 5), 0);
    EXPECT_EQ(net->dist(5, 6), 3);
    EXPECT_EQ(net->dist(5, 7), 2);
    EXPECT_EQ(net->dist(5, 8), 1);

    // node 6
    EXPECT_EQ(net->dist(6, 0), 2);
    EXPECT_EQ(net->dist(6, 1), 3);
    EXPECT_EQ(net->dist(6, 2), 4);
    EXPECT_EQ(net->dist(6, 3), 1);
    EXPECT_EQ(net->dist(6, 4), 2);
    EXPECT_EQ(net->dist(6, 5), 3);
    EXPECT_EQ(net->dist(6, 6), 0);
    EXPECT_EQ(net->dist(6, 7), 1);
    EXPECT_EQ(net->dist(6, 8), 2);

    // node 7
    EXPECT_EQ(net->dist(7, 0), 3);
    EXPECT_EQ(net->dist(7, 1), 2);
    EXPECT_EQ(net->dist(7, 2), 3);
    EXPECT_EQ(net->dist(7, 3), 2);
    EXPECT_EQ(net->dist(7, 4), 1);
    EXPECT_EQ(net->dist(7, 5), 2);
    EXPECT_EQ(net->dist(7, 6), 1);
    EXPECT_EQ(net->dist(7, 7), 0);
    EXPECT_EQ(net->dist(7, 8), 1);

    // node 8
    EXPECT_EQ(net->dist(8, 0), 4);
    EXPECT_EQ(net->dist(8, 1), 3);
    EXPECT_EQ(net->dist(8, 2), 2);
    EXPECT_EQ(net->dist(8, 3), 3);
    EXPECT_EQ(net->dist(8, 4), 2);
    EXPECT_EQ(net->dist(8, 5), 1);
    EXPECT_EQ(net->dist(8, 6), 2);
    EXPECT_EQ(net->dist(8, 7), 1);
    EXPECT_EQ(net->dist(8, 8), 0);
}


TEST(TestNetwork,TestGridNetworkWrapNeighbors) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(true);
    init.set_type(proto::NetworkInit_NetType_GRID);

    std::shared_ptr<Network> net = Network::gen_network(init);

    // node 0
    EXPECT_EQ(net->neigh(0).size(), 4);
    EXPECT_EQ(net->neigh(0).at(0), 1);
    EXPECT_EQ(net->neigh(0).at(1), 2);
    EXPECT_EQ(net->neigh(0).at(2), 3);
    EXPECT_EQ(net->neigh(0).at(3), 6);

    // node 1
    EXPECT_EQ(net->neigh(1).size(), 4);
    EXPECT_EQ(net->neigh(1).at(0), 0);
    EXPECT_EQ(net->neigh(1).at(1), 2);
    EXPECT_EQ(net->neigh(1).at(2), 4);
    EXPECT_EQ(net->neigh(1).at(3), 7);

    // node 2
    EXPECT_EQ(net->neigh(2).size(), 4);
    EXPECT_EQ(net->neigh(2).at(0), 0);
    EXPECT_EQ(net->neigh(2).at(1), 1);
    EXPECT_EQ(net->neigh(2).at(2), 5);
    EXPECT_EQ(net->neigh(2).at(3), 8);

    // node 3
    EXPECT_EQ(net->neigh(3).size(), 4);
    EXPECT_EQ(net->neigh(3).at(0), 0);
    EXPECT_EQ(net->neigh(3).at(1), 4);
    EXPECT_EQ(net->neigh(3).at(2), 5);
    EXPECT_EQ(net->neigh(3).at(3), 6);

    // node 4
    EXPECT_EQ(net->neigh(4).size(), 4);
    EXPECT_EQ(net->neigh(4).at(0), 1);
    EXPECT_EQ(net->neigh(4).at(1), 3);
    EXPECT_EQ(net->neigh(4).at(2), 5);
    EXPECT_EQ(net->neigh(4).at(3), 7);

    // node 5
    EXPECT_EQ(net->neigh(5).size(), 4);
    EXPECT_EQ(net->neigh(5).at(0), 2);
    EXPECT_EQ(net->neigh(5).at(1), 3);
    EXPECT_EQ(net->neigh(5).at(2), 4);
    EXPECT_EQ(net->neigh(5).at(3), 8);

    // node 6
    EXPECT_EQ(net->neigh(6).size(), 4);
    EXPECT_EQ(net->neigh(6).at(0), 0);
    EXPECT_EQ(net->neigh(6).at(1), 3);
    EXPECT_EQ(net->neigh(6).at(2), 7);
    EXPECT_EQ(net->neigh(6).at(3), 8);

    // node 7
    EXPECT_EQ(net->neigh(7).size(), 4);
    EXPECT_EQ(net->neigh(7).at(0), 1);
    EXPECT_EQ(net->neigh(7).at(1), 4);
    EXPECT_EQ(net->neigh(7).at(2), 6);
    EXPECT_EQ(net->neigh(7).at(3), 8);

    // node 8
    EXPECT_EQ(net->neigh(8).size(), 4);
    EXPECT_EQ(net->neigh(8).at(0), 2);
    EXPECT_EQ(net->neigh(8).at(1), 5);
    EXPECT_EQ(net->neigh(8).at(2), 6);
    EXPECT_EQ(net->neigh(8).at(3), 7);
}


TEST(TestNetwork,TestGridNetworkNoWrapNeighbors) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    std::shared_ptr<Network> net = Network::gen_network(init);

    // node 0
    EXPECT_EQ(net->neigh(0).size(), 2);
    EXPECT_EQ(net->neigh(0).at(0), 1);
    EXPECT_EQ(net->neigh(0).at(1), 3);

    // node 1
    EXPECT_EQ(net->neigh(1).size(), 3);
    EXPECT_EQ(net->neigh(1).at(0), 0);
    EXPECT_EQ(net->neigh(1).at(1), 2);
    EXPECT_EQ(net->neigh(1).at(2), 4);

    // node 2
    EXPECT_EQ(net->neigh(2).size(), 2);
    EXPECT_EQ(net->neigh(2).at(0), 1);
    EXPECT_EQ(net->neigh(2).at(1), 5);

    // node 3
    EXPECT_EQ(net->neigh(3).size(), 3);
    EXPECT_EQ(net->neigh(3).at(0), 0);
    EXPECT_EQ(net->neigh(3).at(1), 4);
    EXPECT_EQ(net->neigh(3).at(2), 6);

    // node 4
    EXPECT_EQ(net->neigh(4).size(), 4);
    EXPECT_EQ(net->neigh(4).at(0), 1);
    EXPECT_EQ(net->neigh(4).at(1), 3);
    EXPECT_EQ(net->neigh(4).at(2), 5);
    EXPECT_EQ(net->neigh(4).at(3), 7);

    // node 5
    EXPECT_EQ(net->neigh(5).size(), 3);
    EXPECT_EQ(net->neigh(5).at(0), 2);
    EXPECT_EQ(net->neigh(5).at(1), 4);
    EXPECT_EQ(net->neigh(5).at(2), 8);

    // node 6
    EXPECT_EQ(net->neigh(6).size(), 2);
    EXPECT_EQ(net->neigh(6).at(0), 3);
    EXPECT_EQ(net->neigh(6).at(1), 7);

    // node 7
    EXPECT_EQ(net->neigh(7).size(), 3);
    EXPECT_EQ(net->neigh(7).at(0), 4);
    EXPECT_EQ(net->neigh(7).at(1), 6);
    EXPECT_EQ(net->neigh(7).at(2), 8);

    // node 8
    EXPECT_EQ(net->neigh(8).size(), 2);
    EXPECT_EQ(net->neigh(8).at(0), 5);
    EXPECT_EQ(net->neigh(8).at(1), 7);
}



} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
