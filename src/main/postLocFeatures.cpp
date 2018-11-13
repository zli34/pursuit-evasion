#include "postLocFeatures.hpp"

#include "system.hpp"

#include <njm_cpp/tools/stats.hpp>

namespace coopPE {


PostLocFeatures::PostLocFeatures(
        const std::shared_ptr<const Network> & network,
        const uint32_t & num_pursuers)
    : Features(network), num_pursuers_(num_pursuers),
      num_features_(this->num_pursuers_ * this->network_->size()) {
    std::vector<double> dist_vals;
    for (uint32_t i = 0; i < this->network_->size(); ++i) {
        for (uint32_t j = (i + 1); j < this->network_->size(); ++j) {
            dist_vals.push_back(this->network_->dist(i, j));
        }
    }
    this->dist_scale_ = njm::tools::mean_and_var(dist_vals).second;
}


PostLocFeatures::PostLocFeatures(const PostLocFeatures & other)
    : Features(other), num_pursuers_(other.num_pursuers_),
      num_features_(other.num_features_) {
}


std::shared_ptr<Features> PostLocFeatures::clone() const {
    return std::shared_ptr<Features>(new PostLocFeatures(*this));
}


uint32_t PostLocFeatures::num_features() const {
    return this->num_features_;
}


std::vector<double> PostLocFeatures::get_features(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const std::shared_ptr<const EvaderPost> & post) {
    const std::shared_ptr<EvaderPost> next_post(post->clone());

    auto next_units(curr_units);
    auto next_history(history);
    next_history.push_back(curr_units);
    for (uint32_t i = 0; i < this->num_pursuers_; ++i) {
        System::move_unit(next_units.at(i), this->network_);
    }
    next_post->update(next_units, next_history, tip_history, false, false);

    const std::vector<double> next_probs(next_post->probs());

    std::vector<double> feat;
    feat.reserve(this->num_features_);
    for (uint32_t i = 0; i < this->num_pursuers_; ++i) {
        for (uint32_t j = 0; j < this->network_->size(); ++j) {
            const double dist(this->network_->dist(next_units.at(i).loc(), j));
            feat.push_back(next_probs.at(j) * std::exp(-dist));
        }
    }

    return std::vector<double>();
}


} // namespace coopPE
