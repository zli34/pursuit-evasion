#include "evaderPost.hpp"

namespace coopPE {


EvaderPost::EvaderPost(const std::shared_ptr<const Network> & network)
    : network_(network), prob_history_() {
}


EvaderPost::EvaderPost(const EvaderPost & other)
    : network_(other.network_), prob_history_(other.prob_history_) {
}


std::vector<std::vector<double> > EvaderPost::history() const {
    return this->prob_history_;
}


void EvaderPost::reset() {
    this->prob_history_.clear();
}


void EvaderPost::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
}

} // namespace stdmMf
