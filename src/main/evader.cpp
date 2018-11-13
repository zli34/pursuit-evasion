#include "evader.hpp"

#include <glog/logging.h>

namespace coopPE {


Evader::Evader(const std::shared_ptr<const Network> & network,
        const std::vector<uint32_t> & starting_loc_choices,
        const std::vector<uint32_t> & goal_choices)
    : RngClass(), network_(network),
      starting_loc_choices_(starting_loc_choices), goal_choices_(goal_choices) {
    CHECK_GT(this->starting_loc_choices_.size(), 0);
    CHECK_GT(this->goal_choices_.size(), 0);
}


Evader::Evader(const Evader & other)
    : RngClass(other), network_(other.network_),
      starting_loc_choices_(other.starting_loc_choices_),
      goal_choices_(other.goal_choices_),
      starting_loc_(other.starting_loc_), goal_(other.goal_) {
}


uint32_t Evader::starting_loc() {
    return this->starting_loc_;
}


void Evader::reset() {
    // reset the starting location
    uint32_t choice = this->rng()->rint(0,
            this->starting_loc_choices_.size());
    this->starting_loc_ = this->starting_loc_choices_.at(choice);

    // reset the goal
    choice = this->rng()->rint(0,
            this->goal_choices_.size());
    this->goal_ = this->goal_choices_.at(choice);
}


bool Evader::at_goal(const proto::Unit & curr_unit) const {
    return curr_unit.loc() == this->goal_;
}


void Evader::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
}

} // namespace coopPE
