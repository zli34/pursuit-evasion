#include "runner.hpp"
#include "system.hpp"
#include "network.hpp"

#include "gridWorldPursuer.hpp"
#include "postPeakPursuer.hpp"
#include "futurePeakPursuer.hpp"
#include "qfnRolloutPursuer.hpp"

#include "futurePeakFeatures.hpp"
#include "featMaxPursuer.hpp"
#include "vfnMaxPursuer.hpp"

#include "transMatEvaderPost.hpp"

#include "rwEvader.hpp"

#include "quadrantInformant.hpp"
#include "ewtInformantTimer.hpp"

#include "historian.hpp"

#include <njm_cpp/data/trapperKeeper.hpp>
#include <njm_cpp/info/project.hpp>
#include <njm_cpp/tools/progress.hpp>
#include <njm_cpp/thread/pool.hpp>
#include <thread>

#include <gflags/gflags.h>

DEFINE_int32(num_threads, 0, "Number of threads to use");

using namespace coopPE;

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    proto::NetworkInit init;
    init.set_dim_x(10);
    init.set_dim_y(10);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net = Network::gen_network(init);
    const uint32_t rep = 0;
    const uint32_t num_points = 100;

    std::shared_ptr<njm::tools::Rng> rng(new njm::tools::Rng());
    rng->seed(rep);

    // std::shared_ptr<Features> feat(new FuturePeakFeatures(net, 10));
    // feat->rng(rng);
    std::shared_ptr<EvaderPost> ep(new TransMatEvaderPost(net,
                    {99}, {9, 55, 90}, {0.5, 0.9}));
    ep->rng(rng);
    // GridWorldPursuer p(3, net, {{0}, {1}, {10}}, ep, 0.99);
    // PostPeakPursuer p(3, net, {{0}, {1}, {10}}, ep);
    // FuturePeakPursuer p(3, net, {{0}, {1}, {10}}, ep, 2);
    // FeatMaxPursuer p(3, net, {{0}, {1}, {10}}, ep,
    //         feat, {-1.0});
    // VfnMaxPursuer p(3, net, {{0}, {1}, {10}}, ep, feat, 10, num_points,
    //         10.0, 0.1, 5, 1, 0.5, 0.7);
    QfnRolloutPursuer p(2, net, {{0}, {1}}, ep, 2, 20);
    p.rng(rng);

    RwEvader e(net, {99}, {9, 55, 90}, {0.5, 0.9});
    e.rng(rng);

    EwtInformantTimer eit(0.2,
            std::make_shared<QuadrantInformant>(net));
    eit.rng(rng);

    System s(p.num_units(), net);

    runner(&s, &p, &e, &eit, num_points);

    return 0;
}
