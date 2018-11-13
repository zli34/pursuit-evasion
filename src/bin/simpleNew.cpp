#include "runner.hpp"
#include "system.hpp"
#include "network.hpp"

#include "gridWorldPursuer.hpp"
#include "postPeakPursuerNew.hpp"
#include "futurePeakPursuer.hpp"

#include "futurePeakFeatures.hpp"
#include "featMaxPursuer.hpp"
#include "vfnMaxPursuer.hpp"
#include "qfnRolloutPursuer.hpp"

#include "transMatEvaderPostNew.hpp"

#include "rwEvader.hpp"

#include "quadrantInformantNew.hpp"
#include "ewtInformantTimer.hpp"

#include "historian.hpp"

#include <njm_cpp/data/trapperKeeper.hpp>
#include <njm_cpp/info/project.hpp>
#include <njm_cpp/tools/progress.hpp>
#include <njm_cpp/thread/pool.hpp>
#include <thread>

#include <gflags/gflags.h>

#include<math.h>

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

    std::shared_ptr<Historian> h(new Historian(net));

    const uint32_t num_reps(50);
    const uint32_t num_points(100);

    std::shared_ptr<njm::tools::Progress<std::ostream> >progress(
            new njm::tools::Progress<std::ostream>(num_reps, &std::cout));

    njm::thread::Pool pool(FLAGS_num_threads > 0 ? FLAGS_num_threads :
            std::thread::hardware_concurrency());

    for (uint32_t rep = 0; rep < num_reps; ++rep) {
        pool.service().post([=]() {
            std::shared_ptr<njm::tools::Rng> rng(new njm::tools::Rng());
            rng->seed(rep);

            // std::shared_ptr<Features> feat(new FuturePeakFeatures(net, 10));
            // feat->rng(rng);
            std::shared_ptr<EvaderPost> ep(new TransMatEvaderPost(net,
                            {99}, {0, 7, 70}, {0.75}));
            ep->rng(rng);
            // GridWorldPursuer p(3, net, {{0}, {1}, {10}}, ep, 0.99);
            // PostPeakPursuer p(3, net, {{0}, {1}, {10}}, ep);
            // FuturePeakPursuer p(3, net, {{0}, {1}, {10}}, ep, 2);
            // FeatMaxPursuer p(3, net, {{0}, {1}, {10}}, ep,
            //         feat, {-1.0});
            // VfnMaxPursuer p(3, net, {{0}, {1}, {10}}, ep, feat, 10, num_points,
            //         10.0, 0.1, 5, 1, 0.5, 0.7);
            QfnRolloutPursuer p(2, net, {{0}, {1}}, ep, 2, 10);
            p.rng(rng);

            RwEvader e(net, {99}, {0, 7, 70}, {0.75});
            e.rng(rng);

            EwtInformantTimer eit(0.2,
                    std::make_shared<QuadrantInformant>(net));
            eit.rng(rng);

            System s(p.num_units(), net);

            const proto::Outcome outcome(runner(&s, &p, &e, &eit, num_points));

            std::vector<std::vector<proto::Unit> > pursuer_history(
                    s.pursuer_history());
            pursuer_history.push_back(s.pursuer_units());
            std::vector<proto::Unit> evader_history(s.evader_history());
            evader_history.push_back(s.evader_unit());

            std::vector<std::vector<double> > post_history(ep->history());
            post_history.push_back(*(post_history.end() - 1));

            h->add_rep(evader_history, pursuer_history, post_history,
                    s.tip_history(), outcome, s.time_points());

            progress->update();
        });
    }

    pool.join();
    progress->done();

    njm::data::TrapperKeeper tk(argv[0],
            njm::info::project::PROJECT_ROOT_DIR + "/data");

    h->write_data(*tk.entry("sims.pb"));

    return 0;
}
