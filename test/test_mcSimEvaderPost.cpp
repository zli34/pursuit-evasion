#include <gtest/gtest.h>
#include <glog/logging.h>
#include "network.hpp"
#include "mcSimEvaderPost.hpp"
#include "system.hpp"

namespace coopPE {



TEST(TestMcSimEvaderPost, FullDrift) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    McSimEvaderPost post(net, 10000, {0}, {2}, {1.0});
    post.reset();

    std::map<uint32_t, InformantTip> tip_history;

    std::vector<std::vector<proto::Unit> > pursuer_history;

    // pursuer 1
    std::vector<proto::Unit> pursuer_units;
    pursuer_units.emplace_back();
    proto::Unit & u = pursuer_units.at(0);
    u.set_loc(8);
    u.set_action(proto::Unit_Action_UP);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

    // move
    System::move_unit(u, net);
    pursuer_history.push_back(pursuer_units);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(1), 1.0, 1e-10);
}

TEST(TestMcSimEvaderPost, HalfDrift) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    McSimEvaderPost post(net, 10000, {0}, {2}, {0.5});
    post.reset();

    std::map<uint32_t, InformantTip> tip_history;

    std::vector<std::vector<proto::Unit> > pursuer_history;

    // pursuer 1
    std::vector<proto::Unit> pursuer_units;
    pursuer_units.emplace_back();
    proto::Unit & u = pursuer_units.at(0);
    u.set_loc(8);
    u.set_action(proto::Unit_Action_UP);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

    // move
    System::move_unit(u, net);
    pursuer_history.push_back(pursuer_units);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 0.6 * 0.5, 1e-2);
    EXPECT_NEAR(post.prob(1), 0.2 * 0.5 + 1.0 * 0.5, 1e-2);
    EXPECT_NEAR(post.prob(3), 0.2 * 0.5, 1e-2);
}


TEST(TestMcSimEvaderPost, NoDrift) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    McSimEvaderPost post(net, 10000, {0}, {2}, {0.0});
    post.reset();

    std::map<uint32_t, InformantTip> tip_history;

    std::vector<std::vector<proto::Unit> > pursuer_history;

    // pursuer 1
    std::vector<proto::Unit> pursuer_units;
    pursuer_units.emplace_back();
    proto::Unit & u = pursuer_units.at(0);
    u.set_loc(8);
    u.set_action(proto::Unit_Action_UP);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

    // move
    System::move_unit(u, net);
    pursuer_history.push_back(pursuer_units);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 0.6, 1e-2);
    EXPECT_NEAR(post.prob(1), 0.2, 1e-2);
    EXPECT_NEAR(post.prob(3), 0.2, 1e-2);
}


TEST(TestMcSimEvaderPost, NoDriftCatch) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    McSimEvaderPost post(net, 10000, {0}, {2}, {0.0});
    post.reset();

    std::map<uint32_t, InformantTip> tip_history;

    std::vector<std::vector<proto::Unit> > pursuer_history;

    // pursuer 1
    std::vector<proto::Unit> pursuer_units;
    pursuer_units.emplace_back();
    proto::Unit & u = pursuer_units.at(0);
    u.set_loc(5);
    u.set_action(proto::Unit_Action_LEFT);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

    // move
    pursuer_history.push_back(pursuer_units);
    System::move_unit(u, net);

    // update
    post.update(pursuer_units, pursuer_history, tip_history, false, true);

    EXPECT_NEAR(post.prob(0), 0.75, 1e-2);
    EXPECT_NEAR(post.prob(1), 0.0, 1e-2);
    EXPECT_NEAR(post.prob(3), 0.25, 1e-2);
}

TEST(TestMcSimEvaderPost, FullDriftCatchThrow) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    { // throw on first step
        McSimEvaderPost post(net, 10000, {0}, {2}, {1.0});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(3);
        u.set_action(proto::Unit_Action_LEFT);

        // update
        EXPECT_THROW(
                try {
                    post.update(pursuer_units, pursuer_history,
                            tip_history, false, true);
                } catch (const std::runtime_error & e) {
                    EXPECT_STREQ(e.what(), "Do-over too many times");
                    throw;
                },
                std::runtime_error);
    }

    { // throw on second step
        McSimEvaderPost post(net, 10000, {0}, {2}, {1.0});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(5);
        u.set_action(proto::Unit_Action_UP);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);
        EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

        // move
        pursuer_history.push_back(pursuer_units);
        System::move_unit(u, net);

        // update
        EXPECT_THROW(
                try {
                    post.update(pursuer_units, pursuer_history,
                            tip_history, false, true);
                } catch (const std::runtime_error & e) {
                    EXPECT_STREQ(e.what(), "Do-over too many times");
                    throw;
                },
                std::runtime_error);

    }
}



TEST(TestMcSimEvaderPost, OneStepInformant) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    { // reliable w/ half drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.5});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);
        u.set_action(proto::Unit_Action_UP);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

        // move
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 1.0, 0.0, 0.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 0.75, 1e-2);
        EXPECT_NEAR(post.prob(3), 0.25, 1e-2);
    }

    { // deceitful w/ half drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.5});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);
        u.set_action(proto::Unit_Action_UP);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

        // move
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 0.0, 1.0, 0.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(1), 1.0, 1e-10);

    }

    { // noisy w/ half drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.5});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);
        u.set_action(proto::Unit_Action_UP);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

        // move
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 0.0, 0.0, 1.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 0.5 * 0.6, 1e-2);
        EXPECT_NEAR(post.prob(1), 0.5 + 0.5 * 0.2, 1e-2);
        EXPECT_NEAR(post.prob(3), 0.5 * 0.2, 1e-2);
    }

    { // mixture w/ no drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.0});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);
        u.set_action(proto::Unit_Action_UP);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

        // move
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 0.35, 0.45, 0.2));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        const double prob_feasible(
                0.6 * 0.35 + // E^1 = 0 and reliable
                0.6 * 0.2 + // E^1 = 0 and noisy
                0.2 * 0.45 + // E^1 = 1 and deceitful
                0.2 * 0.2 + // E^1 = 1 and noisy
                0.2 * 0.35 + // E^1 = 3 and reliable
                0.2 * 0.2); // E^1 = 3 and noisy

        const double prob_0((0.6 * 0.35 + 0.6 * 0.2) / prob_feasible);

        const double prob_1((0.2 * 0.45 + 0.2 * 0.2) / prob_feasible);

        const double prob_3((0.2 * 0.35 + 0.2 * 0.2) / prob_feasible);

        EXPECT_NEAR(post.prob(0), prob_0, 1e-2);
        EXPECT_NEAR(post.prob(1), prob_1, 1e-2);
        EXPECT_NEAR(post.prob(3), prob_3, 1e-2);

        const std::vector<double> probs(post.probs());
        EXPECT_NEAR(std::accumulate(probs.begin(), probs.end(), 0.0),
                1.0, 1e-3);
    }

    { // mixture w/ half drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.5});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);
        u.set_action(proto::Unit_Action_UP);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        EXPECT_NEAR(post.prob(0), 1.0, 1e-10);

        // move
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 0.35, 0.45, 0.2));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        const double prob_feasible(
                0.5 * 0.6 * 0.35 + // E^1 = 0 and reliable
                0.5 * 0.6 * 0.2 + // E^1 = 0 and noisy
                (0.5 + 0.5 * 0.2) * 0.45 + // E^1 = 1 and deceitful
                (0.5 + 0.5 * 0.2) * 0.2 + // E^1 = 1 and noisy
                0.5 * 0.2 * 0.35 + // E^1 = 3 and reliable
                0.5 * 0.2 * 0.2); // E^1 = 3 and noisy

        const double prob_0((
                        0.5 * 0.6 * 0.35 +
                        0.5 * 0.6 * 0.2)
                / prob_feasible);

        const double prob_1((
                        (0.5 + 0.5 * 0.2) * 0.45 +
                        (0.5 + 0.5 * 0.2) * 0.2)
                / prob_feasible);

        const double prob_3((
                        0.5 * 0.2 * 0.35 +
                        0.5 * 0.2 * 0.2)
                / prob_feasible);

        EXPECT_NEAR(post.prob(0), prob_0, 1e-2);
        EXPECT_NEAR(post.prob(1), prob_1, 1e-2);
        EXPECT_NEAR(post.prob(3), prob_3, 1e-2);

        const std::vector<double> probs(post.probs());
        EXPECT_NEAR(std::accumulate(probs.begin(), probs.end(), 0.0),
                1.0, 1e-3);
    }
}




TEST(TestMcSimEvaderPost, TwoStepInformant) {
    proto::NetworkInit init;
    init.set_dim_x(3);
    init.set_dim_y(3);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net(Network::gen_network(init));

    { // reliable w/ no drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.0});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        // move
        u.set_action(proto::Unit_Action_UP);
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 1.0, 0.0, 0.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        // check one step values
        EXPECT_NEAR(post.prob(0), 0.75, 1e-2);
        EXPECT_NEAR(post.prob(3), 0.25, 1e-2);

        // move
        u.set_action(proto::Unit_Action_DOWN);
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(2, InformantTip({1, 4}, 1.0, 0.0, 0.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);


        // check one step values
        EXPECT_NEAR(post.prob(1), 0.75, 1e-2);
        EXPECT_NEAR(post.prob(4), 0.25, 1e-2);
    }


    { // deceitful w/ no drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.0});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        // move
        u.set_action(proto::Unit_Action_UP);
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 0.0, 1.0, 0.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        // check one step values
        EXPECT_NEAR(post.prob(1), 1.0, 1e-2);

        // move
        u.set_action(proto::Unit_Action_DOWN);
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(2, InformantTip({1, 4}, 0.0, 1.0, 0.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);


        // check one step values
        EXPECT_NEAR(post.prob(0), 0.5, 1e-2);
        EXPECT_NEAR(post.prob(2), 0.5, 1e-2);
    }


    { // noisy w/ no drift
        McSimEvaderPost post(net, 10000, {0}, {2}, {0.0});
        post.reset();

        std::map<uint32_t, InformantTip> tip_history;

        std::vector<std::vector<proto::Unit> > pursuer_history;

        // pursuer 1
        std::vector<proto::Unit> pursuer_units;
        pursuer_units.emplace_back();
        proto::Unit & u = pursuer_units.at(0);
        u.set_loc(8);

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        // move
        u.set_action(proto::Unit_Action_UP);
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(1, InformantTip({0, 3}, 0.0, 0.0, 1.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);

        // check one step values
        EXPECT_NEAR(post.prob(0), 0.6, 1e-2);
        EXPECT_NEAR(post.prob(1), 0.2, 1e-2);
        EXPECT_NEAR(post.prob(3), 0.2, 1e-2);

        // move
        u.set_action(proto::Unit_Action_DOWN);
        System::move_unit(u, net);
        pursuer_history.push_back(pursuer_units);

        // add informant
        tip_history.emplace(2, InformantTip({1, 4}, 0.0, 0.0, 1.0));

        // update
        post.update(pursuer_units, pursuer_history, tip_history, false, true);


        // check one step values
        EXPECT_NEAR(post.prob(0), 0.44, 1e-2);
        EXPECT_NEAR(post.prob(1), 0.2, 1e-2);
        EXPECT_NEAR(post.prob(2), 0.04, 1e-2);
        EXPECT_NEAR(post.prob(3), 0.2, 1e-2);
        EXPECT_NEAR(post.prob(4), 0.08, 1e-2);
        EXPECT_NEAR(post.prob(6), 0.04, 1e-2);
    }
}




} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
