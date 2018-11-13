#include "featMaxPursuer.hpp"

#include <njm_cpp/linalg/stdVectorAlgebra.hpp>

#include <glog/logging.h>

namespace coopPE {

FeatMaxPursuer::FeatMaxPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices,
        const std::shared_ptr<EvaderPost> & evader_post,
        const std::shared_ptr<Features> & features,
        const std::vector<double> & coef)
    : Pursuer(num_units, network, starting_loc_choices),
      evader_post_(evader_post), features_(features), coef_(coef) {
    this->evader_post_->rng(this->rng());
}


FeatMaxPursuer::FeatMaxPursuer(const FeatMaxPursuer & other)
    : Pursuer(other), evader_post_(other.evader_post_->clone()),
      features_(other.features_->clone()) {
    this->evader_post_->rng(this->rng());
}


std::shared_ptr<Pursuer> FeatMaxPursuer::clone() const {
    return std::shared_ptr<Pursuer>(new FeatMaxPursuer(*this));
}


std::vector<proto::Unit_Action> FeatMaxPursuer::decide_moves(
        const std::vector<proto::Unit> &curr_units,
        const std::vector<std::vector<proto::Unit> > &history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    // update posterior
    this->evader_post_->update(curr_units, history, tip_history, false, false);

    // get action sets that maximize linear combination of features
    const std::vector<std::vector<proto::Unit_Action> > maximizing_actions(
            this->features_->arg_max(curr_units, history, tip_history,
                    this->evader_post_, this->coef_, njm::linalg::dot_a_and_b));

    CHECK_GT(maximizing_actions.size(), 0);

    // break ties uniformly if any
    if (maximizing_actions.size() > 1) {
        return maximizing_actions.at(this->rng()->rint(0,
                        maximizing_actions.size()));
    } else {
        return maximizing_actions.at(0);
    }
}


void FeatMaxPursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


void FeatMaxPursuer::rng(const std::shared_ptr<njm::tools::Rng> &rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
}




} // namespace coopPE
