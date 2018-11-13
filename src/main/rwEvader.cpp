#include "rwEvader.hpp"

#include <glog/logging.h>

namespace coopPE {


RwEvader::RwEvader(const std::shared_ptr<const Network> & network,
        const std::vector<uint32_t> & starting_loc_choices,
        const std::vector<uint32_t> & goal_choices,
        const std::vector<double> & drift_choices)
    : Evader(network, starting_loc_choices, goal_choices),
      drift_choices_(drift_choices) {
    CHECK_GT(this->drift_choices_.size(), 0);
}


RwEvader::RwEvader(const RwEvader & other)
    : Evader(other), drift_choices_(other.drift_choices_),
      drift_(other.drift_) {
}


std::shared_ptr<Evader> RwEvader::clone() const {
    return std::shared_ptr<RwEvader>(new RwEvader(*this));
}


proto::Unit_Action RwEvader::decide_move(const proto::Unit & curr_unit,
        const std::vector<proto::Unit> & history) {
    if (this->rng()->runif_01() < this->drift_) {
        return this->drift_move(curr_unit, history);
    } else {
        return this->random_move(curr_unit, history);
    }
}


proto::Unit_Action RwEvader::drift_move(const proto::Unit & curr_unit,
        const std::vector<proto::Unit> & history) {
    const proto::Node & loc = this->network_->node(curr_unit.loc());

    // set nothing as first option
    uint32_t best_dist = this->network_->dist(this->goal_, loc.index());
    std::vector<proto::Unit_Action> best_actions({proto::Unit_Action_NOTHING});

    // check up
    uint32_t cand_dist = this->network_->dist(this->goal_, loc.up());
    if (cand_dist < best_dist) {
        best_dist = cand_dist;
        best_actions.clear();
        best_actions.push_back(proto::Unit_Action_UP);
    } else if(cand_dist == best_dist) {
        best_actions.push_back(proto::Unit_Action_UP);
    }

    // check down
    cand_dist = this->network_->dist(this->goal_, loc.down());
    if (cand_dist < best_dist) {
        best_dist = cand_dist;
        best_actions.clear();
        best_actions.push_back(proto::Unit_Action_DOWN);
    } else if(cand_dist == best_dist) {
        best_actions.push_back(proto::Unit_Action_DOWN);
    }

    // check left
    cand_dist = this->network_->dist(this->goal_, loc.left());
    if (cand_dist < best_dist) {
        best_dist = cand_dist;
        best_actions.clear();
        best_actions.push_back(proto::Unit_Action_LEFT);
    } else if(cand_dist == best_dist) {
        best_actions.push_back(proto::Unit_Action_LEFT);
    }

    // check right
    cand_dist = this->network_->dist(this->goal_, loc.right());
    if (cand_dist < best_dist) {
        best_dist = cand_dist;
        best_actions.clear();
        best_actions.push_back(proto::Unit_Action_RIGHT);
    } else if(cand_dist == best_dist) {
        best_actions.push_back(proto::Unit_Action_RIGHT);
    }

    CHECK_GT(best_actions.size(), 0) << "best actions has length 0";
    // return best action
    if (best_actions.size() > 1) {
        return best_actions.at(this->rng()->rint(0, best_actions.size()));
    } else {
        return best_actions.at(0);
    }
}


proto::Unit_Action RwEvader::random_move(const proto::Unit & curr_unit,
        const std::vector<proto::Unit> & history) {
    return static_cast<proto::Unit_Action>(this->rng()->rint(
                    proto::Unit_Action_Action_MIN,
                    proto::Unit_Action_Action_ARRAYSIZE));
}


void RwEvader::reset() {
    this->Evader::reset();
    this->drift_ = this->drift_choices_.at(this->rng()->rint(0,
                    this->drift_choices_.size()));
}


} // namespace coopPE
