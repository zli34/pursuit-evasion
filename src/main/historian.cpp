#include "historian.hpp"

namespace coopPE {


Historian::Historian(const std::shared_ptr<const Network> & network)
    : network_(network), num_reps_(0) {
    *this->ssd_.mutable_network() = this->network_->node_list();

}


uint32_t Historian::add_rep(const std::vector<proto::Unit> & evader_history,
        const std::vector<std::vector<proto::Unit> > & pursuer_history,
        const std::vector<std::vector<double> > & evader_post_history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const proto::Outcome & outcome, const uint32_t & num_points) {
    std::lock_guard<std::mutex> lock(this->mtx_);

    // add 1 for the initial state
    CHECK_EQ(evader_history.size(), num_points + 1);
    CHECK_EQ(pursuer_history.size(), num_points + 1);
    CHECK_EQ(evader_post_history.size(), num_points + 1);

    proto::RepData * const rd(this->ssd_.add_reps());
    rd->set_outcome(outcome);

    for (uint32_t i = 0; i < (num_points + 1); ++i) {
        proto::TimePointData * const td(rd->add_time_points());

        // add pursuers
        std::for_each(pursuer_history.at(i).begin(),
                pursuer_history.at(i).end(),
                [&] (const proto::Unit & unit) {
                    *td->add_pursuers() = unit;
                });

        if (tip_history.find(i) != tip_history.end()) {
            const InformantTip & tip(tip_history.find(i)->second);
            td->mutable_informant()->set_has_tip(tip.has_tip());

            // locations
            std::for_each(tip.locs().begin(), tip.locs().end(),
                    [&] (const uint32_t & loc) {
                        td->mutable_informant()->add_locs(loc);
                    });

            // distribution
            td->mutable_informant()->set_reliable(tip.reliable());
            td->mutable_informant()->set_deceitful(tip.deceitful());
            td->mutable_informant()->set_noisy(tip.noisy());
        }

        // posterior
        std::for_each(evader_post_history.at(i).begin(),
                evader_post_history.at(i).end(),
                [&] (const double & prob) {
                    td->add_posterior(prob);
                });

        // evader
        *td->mutable_evader() = evader_history.at(i);
    }

    return this->num_reps_++;
}


} // namespace coopPE
