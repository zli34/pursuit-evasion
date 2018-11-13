#include <gtest/gtest.h>
#include <glog/logging.h>
#include "network.hpp"
#include "system.hpp"

namespace coopPE {



TEST(TestSystem, Move) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    proto::Unit unit;

    // location 0
    unit.set_loc(0);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 0);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(0);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 1);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(0);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 0);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(0);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 3);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(0);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 0);
    EXPECT_FALSE(unit.has_action());


    // location 1
    unit.set_loc(1);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 0);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(1);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 2);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(1);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 1);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(1);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 4);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(1);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 1);
    EXPECT_FALSE(unit.has_action());

    // location 2
    unit.set_loc(2);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 1);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(2);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 2);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(2);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 2);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(2);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 5);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(2);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 2);
    EXPECT_FALSE(unit.has_action());

    // location 3
    unit.set_loc(3);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 3);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(3);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 4);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(3);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 0);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(3);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 6);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(3);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 3);
    EXPECT_FALSE(unit.has_action());

    // location 4
    unit.set_loc(4);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 3);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(4);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 5);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(4);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 1);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(4);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 7);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(4);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 4);
    EXPECT_FALSE(unit.has_action());

    // location 5
    unit.set_loc(5);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 4);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(5);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 5);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(5);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 2);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(5);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 8);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(5);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 5);
    EXPECT_FALSE(unit.has_action());

    // location 6
    unit.set_loc(6);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 6);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(6);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 7);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(6);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 3);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(6);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 6);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(6);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 6);
    EXPECT_FALSE(unit.has_action());

    // location 7
    unit.set_loc(7);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 6);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(7);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 8);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(7);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 4);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(7);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 7);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(7);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 7);
    EXPECT_FALSE(unit.has_action());

    // location 8
    unit.set_loc(8);
    unit.set_action(proto::Unit_Action_UP);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 7);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(8);
    unit.set_action(proto::Unit_Action_DOWN);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 8);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(8);
    unit.set_action(proto::Unit_Action_LEFT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 5);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(8);
    unit.set_action(proto::Unit_Action_RIGHT);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 8);
    EXPECT_FALSE(unit.has_action());

    unit.set_loc(8);
    unit.set_action(proto::Unit_Action_NOTHING);
    System::move_unit(unit, net);
    EXPECT_EQ(unit.loc(), 8);
    EXPECT_FALSE(unit.has_action());
}




} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
