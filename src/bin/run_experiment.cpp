#include "runner.hpp"
#include "system.hpp"
#include "network.hpp"

#include "gridWorldPursuer.hpp"
#include "postPeakPursuer.hpp"
#include "futurePeakPursuer.hpp"

#include "futurePeakFeatures.hpp"
#include "featMaxPursuer.hpp"
#include "qfnRolloutPursuer.hpp"

#include "rwPursuer.hpp"

#include "transMatEvaderPost.hpp"

#include "rwEvader.hpp"

#include "quadrantInformant.hpp"
#include "ewtInformantTimer.hpp"
#include "nullInformantTimer.hpp"

#include "historian.hpp"

#include <njm_cpp/data/trapperKeeper.hpp>
#include <njm_cpp/info/project.hpp>
#include <njm_cpp/tools/progress.hpp>
#include <njm_cpp/thread/pool.hpp>
#include <njm_cpp/tools/experiment.hpp>
#include <thread>

#include <gflags/gflags.h>


DEFINE_int32(num_threads, 0, "Number of threads to use");

using namespace coopPE;

using njm::tools::Experiment;

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    proto::NetworkInit init;
    init.set_dim_x(10);
    init.set_dim_y(10);
    init.set_wrap(false);
    init.set_type(proto::NetworkInit_NetType_GRID);

    const std::shared_ptr<const Network> net = Network::gen_network(init);

    std::shared_ptr<Historian> qfn_rollout_historian(new Historian(net));
    std::shared_ptr<Historian> rw_historian(new Historian(net));

    const uint32_t num_reps(2000);
    const uint32_t num_points(100);

    std::shared_ptr<njm::tools::Progress<std::ostream> >progress(
            new njm::tools::Progress<std::ostream>(num_reps, &std::cout));

    njm::thread::Pool pool(FLAGS_num_threads > 0 ? FLAGS_num_threads :
            std::thread::hardware_concurrency());


    Experiment e;

    {
        Experiment::FactorGroup * g = e.add_group();

        g->add_factor(std::vector<int>({1, 2, 3})); // num_pursuers
        g->add_factor(std::vector<double>(
            {0.1, 0.5, 1.0})); // informant rate
        g->add_factor(std::vector<int>({0, 1, 2})); // rollout steps
    }

    std::map<uint32_t, std::vector<std::vector<uint32_t> > > starting_locs;
    starting_locs[1] = {{0}};
    starting_locs[2] = {{0}, {1}};
    starting_locs[3] = {{0}, {1}, {10}};

    njm::data::TrapperKeeper tk(argv[0],
            njm::info::project::PROJECT_ROOT_DIR + "/data");

    njm::data::Entry * const results(tk.entry("outcomes.csv"));
    *results << "pursuer,"
             << "level,"
             << "rep,"
             << "sim_num,"
             << "num_pursuers,"
             << "informant_rate,"
             << "rollout_steps,"
             << "time_points,"
             << "outcome\n";

    uint32_t num_jobs(0);
    uint32_t level_num(0);
    e.start();
    do {
        const Experiment::Factor f = e.get();

        for (uint32_t rep = 0; rep < num_reps; ++rep) {
            uint32_t i = 0;
            CHECK_EQ(f.at(i).type, Experiment::FactorLevel::Type::is_int);
            const uint32_t num_pursuers = static_cast<uint32_t>(
                    f.at(i++).val.int_val);
            CHECK_EQ(f.at(i).type, Experiment::FactorLevel::Type::is_double);
            const double informant_rate = f.at(i++).val.double_val;
            CHECK_EQ(f.at(i).type, Experiment::FactorLevel::Type::is_int);
            const uint32_t rollout_steps = static_cast<uint32_t>(
                    f.at(i++).val.int_val);

            // check number of factors
            CHECK_EQ(i, f.size());

            pool.service().post([=]() {
                std::shared_ptr<njm::tools::Rng> rng(new njm::tools::Rng());
                rng->seed(rep);

                std::shared_ptr<EvaderPost> ep(new TransMatEvaderPost(net,
                                {99}, {0, 7, 70}, {0.65, 0.75, 0.85}));
                ep->rng(rng);

                QfnRolloutPursuer p(num_pursuers, net,
                        starting_locs.at(num_pursuers), ep, rollout_steps, 20);
                p.rng(rng);

                RwEvader e(net, {99}, {0, 7, 70}, {0.65, 0.75, 0.85});
                e.rng(rng);

                EwtInformantTimer eit(informant_rate,
                        std::make_shared<QuadrantInformant>(net));
                eit.rng(rng);

                System s(p.num_units(), net);

                const proto::Outcome outcome(runner(&s, &p, &e, &eit,
                                num_points));

                std::vector<std::vector<proto::Unit> > pursuer_history(
                        s.pursuer_history());
                pursuer_history.push_back(s.pursuer_units());
                std::vector<proto::Unit> evader_history(s.evader_history());
                evader_history.push_back(s.evader_unit());

                std::vector<std::vector<double> > post_history(ep->history());
                post_history.push_back(*(post_history.end() - 1));

                const uint32_t sim_num = qfn_rollout_historian->add_rep(
                        evader_history, pursuer_history, post_history,
                        s.tip_history(), outcome, s.time_points());

                std::stringstream ss;
                ss << "qfn_rollout" << ","
                   << level_num << ","
                   << rep << ","
                   << sim_num << ","
                   << num_pursuers << ","
                   << informant_rate << ","
                   << rollout_steps << ","
                   << s.time_points() << ","
                   << proto::Outcome_Name(outcome) << "\n";

                *results << ss.str();

                progress->update();
            });

            ++num_jobs;
        }

        ++level_num;
    } while (e.next());


    e = Experiment();

    {
        Experiment::FactorGroup * g = e.add_group();

        g->add_factor(std::vector<int>({1, 2, 3})); // num_pursuers
    }


    level_num = 0;
    e.start();
    do {
        const Experiment::Factor f = e.get();

        for (uint32_t rep = 0; rep < num_reps; ++rep) {
            uint32_t i = 0;
            CHECK_EQ(f.at(i).type, Experiment::FactorLevel::Type::is_int);
            const uint32_t num_pursuers = static_cast<uint32_t>(
                    f.at(i++).val.int_val);

            // check number of factors
            CHECK_EQ(i, f.size());

            pool.service().post([=]() {
                std::shared_ptr<njm::tools::Rng> rng(new njm::tools::Rng());
                rng->seed(rep);

                RwPursuer p(num_pursuers, net, starting_locs.at(num_pursuers));
                p.rng(rng);

                RwEvader e(net, {99}, {0, 7, 70}, {0.75});
                e.rng(rng);

                NullInformantTimer nit;

                System s(p.num_units(), net);

                const proto::Outcome outcome(runner(&s, &p, &e, &nit,
                                num_points));

                std::vector<std::vector<proto::Unit> > pursuer_history(
                        s.pursuer_history());
                pursuer_history.push_back(s.pursuer_units());
                std::vector<proto::Unit> evader_history(s.evader_history());
                evader_history.push_back(s.evader_unit());

                std::vector<std::vector<double> > post_history(
                        s.time_points() + 1,
                        std::vector<double>(net->size(), 0.0));

                const uint32_t sim_num = rw_historian->add_rep(evader_history,
                        pursuer_history, post_history, s.tip_history(),
                        outcome, s.time_points());

                std::stringstream ss;
                ss << "rw" << ","
                   << level_num << ","
                   << rep << ","
                   << sim_num << ","
                   << num_pursuers << ","
                   << "NA" << ","
                   << "NA" << ","
                   << s.time_points() << ","
                   << proto::Outcome_Name(outcome) << "\n";

                *results << ss.str();

                progress->update();
            });

            ++num_jobs;
        }

        ++level_num;
    } while (e.next());


    progress->total(num_jobs);

    pool.join();
    progress->done();

    qfn_rollout_historian->write_data(*tk.entry("qfn_rollout_sims.pb"));
    rw_historian->write_data(*tk.entry("rw_sims.pb"));

    return 0;
}
