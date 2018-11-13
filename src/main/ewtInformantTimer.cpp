#include "ewtInformantTimer.hpp"


namespace coopPE {


EwtInformantTimer::EwtInformantTimer(
        const double & rate, const std::shared_ptr<Informant> & informant)
    : InformantTimer(), rate_(rate), informant_(informant), num_points_(0),
      threshold_(0.0) {
    this->informant_->rng(this->rng());
}


InformantTip EwtInformantTimer::generate_tip(const proto::Unit & evader_unit) {
    if (this->num_points_ >= this->threshold_) {
        // increase threshold
        this->threshold_ += this->rng()->rexp(this->rate_);

        // increase points
        ++this->num_points_;

        // create informant
        return this->informant_->tip(evader_unit);
    } else {
        // increase points
        ++this->num_points_;

        // return empty informant
        return InformantTip();
    }
}


void EwtInformantTimer::reset() {
    this->threshold_ = this->rng()->rexp(this->rate_);
    this->num_points_ = 0;
}

void EwtInformantTimer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
    this->informant_->rng(rng);
}


} // namespace coopPE
