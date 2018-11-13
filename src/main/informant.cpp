#include "informant.hpp"

namespace coopPE {


InformantTip::InformantTip()
    : has_tip_(false), locs_(), reliable_(0.0), deceitful_(0.0), noisy_(1.0) {
}


InformantTip::InformantTip(
        const std::vector<uint32_t> & locs,
        const double & reliable,
        const double & deceitful,
        const double & noisy)
    : has_tip_(true), locs_(locs), reliable_(reliable), deceitful_(deceitful),
      noisy_(noisy) {

    CHECK_GT(this->locs_.size(), 0);
    CHECK_GE(this->reliable_, 0.0);
    CHECK_GE(this->deceitful_, 0.0);
    CHECK_GE(this->noisy_, 0.0);
    CHECK_NEAR(this->reliable_ + this->deceitful_ + this->noisy_, 1.0, 1e-6);
}


InformantTip::InformantTip(const InformantTip & other)
    : has_tip_(other.has_tip_), locs_(other.locs_), reliable_(other.reliable_),
      deceitful_(other.deceitful_), noisy_(other.noisy_) {
}


Informant::Informant(const std::shared_ptr<const Network> & network)
    : network_(network) {
}


Informant::Informant(const Informant & other)
    : network_(other.network_) {
}


void Informant::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
}


} // namespace coopPE
