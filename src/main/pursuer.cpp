#include "pursuer.hpp"

#include <glog/logging.h>

namespace coopPE {


Pursuer::Pursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_locs_choices)
    : num_units_(num_units), starting_locs_choices_(starting_locs_choices),
      network_(network) {

    CHECK_EQ(num_units, this->starting_locs_choices_.size());
    for (uint32_t i = 0; i < this->starting_locs_choices_.size(); ++i) {
        CHECK_GT(this->starting_locs_choices_.at(i).size(), 0);
    }
}


Pursuer::Pursuer(const Pursuer & other)
    : RngClass(other), num_units_(other.num_units_),
      starting_locs_choices_(other.starting_locs_choices_),
      network_(other.network_), starting_locs_(other.starting_locs_) {
}


std::vector<uint32_t> Pursuer::starting_locs() {
    return this->starting_locs_;
}


void Pursuer::reset() {
    // rest the starting locations
    this->starting_locs_.clear();
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        const std::vector<uint32_t> & locs = this->starting_locs_choices_.at(i);
        const uint32_t choice = this->rng()->rint(0, locs.size());
        this->starting_locs_.push_back(locs.at(choice));
    }
}


uint32_t Pursuer::num_units() const {
    return this->num_units_;
}


void Pursuer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
}


} // namespace coopPE
