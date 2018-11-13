#include "bruteForcePursuer.hpp"

#include <njm_cpp/linalg/stdVectorAlgebra.hpp>

#include <glog/logging.h>

namespace coopPE {


BruteForcePursuer::BruteForcePursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices,
        const std::shared_ptr<EvaderPost> & evader_post,
        const std::shared_ptr<Features> & features,
        const std::vector<double> & par)
    : Pursuer(num_units, network, starting_loc_choices),
      evader_post_(evader_post), features_(features), par_(par) {

    CHECK_EQ(this->par_.size(), this->features_->num_features());

    this->evader_post_->rng(this->rng());
    this->features_->rng(this->rng());
}


BruteForcePursuer::BruteForcePursuer(const BruteForcePursuer & other)
    : Pursuer(other), evader_post_(other.evader_post_->clone()),
      features_(other.features_->clone()), par_(other.par_) {

    this->evader_post_->rng(this->rng());
    this->features_->rng(this->rng());
}


std::shared_ptr<Pursuer> BruteForcePursuer::clone() const {
    return std::shared_ptr<Pursuer>(new BruteForcePursuer(*this));
}


std::vector<proto::Unit_Action> BruteForcePursuer::decide_moves(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    // update posterior
    this->evader_post_->update(curr_units, history, tip_history, false, false);

    // need to copy so we can modify actions
    std::vector<proto::Unit> units(curr_units);

    std::vector<proto::Unit_Action> actions(this->num_units_,
            proto::Unit_Action_NOTHING);

    std::vector<std::vector<proto::Unit_Action> > best_actions;
    double best_value(std::numeric_limits<double>::lowest());

    // iterate over all actions and find the best one
    do {
        // set actions
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            units.at(i).set_action(actions.at(i));
        }

        // get features for set of actions
        const std::vector<double> f(this->features_->get_features(units,
                        history, tip_history, this->evader_post_));

        // evaluate
        const double v(njm::linalg::dot_a_and_b(this->par_, f));

        // check if larger or equal value
        if (v > best_value) {
            best_actions.clear();
            best_actions.push_back(actions);
            best_value = v;
        } else if(v == best_value) {
            best_actions.push_back(actions);
        }

    } while(this->next_move(actions));

    return best_actions.at(this->rng()->rint(0, best_actions.size()));
}


bool BruteForcePursuer::next_move(
        std::vector<proto::Unit_Action> & moves) const {
    CHECK_EQ(moves.size(), this->num_units_);
    // increment action and return, if can't increment (already at
    // RIGHT) then reset (set to NOTHING) and move to next pursuer
    // (don't return immediately)
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        if (moves.at(i) == proto::Unit_Action_NOTHING) {
            moves.at(i) = proto::Unit_Action_UP;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_UP) {
            moves.at(i) = proto::Unit_Action_DOWN;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_DOWN) {
            moves.at(i) = proto::Unit_Action_LEFT;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_LEFT) {
            moves.at(i) = proto::Unit_Action_RIGHT;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_RIGHT) {
            moves.at(i) = proto::Unit_Action_NOTHING;
        } else {
            LOG(FATAL) << "Unknown value for action: " << moves.at(i);
        }
    }
    // all pursuers were already set to RIGHT, this is the last set of
    // actions, thus return false to indicate no more moves,
    return false;
}


void BruteForcePursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


void BruteForcePursuer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
    this->features_->rng(rng);
}





} // namespace coopPE
