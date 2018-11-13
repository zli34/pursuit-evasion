#include "futurePeakFeatures.hpp"

#include "system.hpp"

#include "utilities.hpp"

#include <queue>

#include <glog/logging.h>


namespace coopPE {


FuturePeakFeatures::FuturePeakFeatures(
        const std::shared_ptr<const Network> & network,
        const uint32_t & future_steps)
    : Features(network), future_steps_(future_steps),
      num_features_(1 + future_steps) {
}


FuturePeakFeatures::FuturePeakFeatures(const FuturePeakFeatures & other)
    : Features(other), future_steps_(other.future_steps_),
      num_features_(other.num_features_) {
}


std::shared_ptr<Features> FuturePeakFeatures::clone() const {
    return std::shared_ptr<Features>(new FuturePeakFeatures(*this));
}


std::vector<double> FuturePeakFeatures::get_features(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const std::shared_ptr<const EvaderPost> & post) {

    std::vector<double> features;

    std::vector<proto::Unit> moved_units(curr_units);
    std::for_each(moved_units.begin(), moved_units.end(),
            [this] (proto::Unit & u) {
                System::move_unit(u, this->network_);
            });

    // with no look ahead
    features.push_back(this->dist_to_peaks(moved_units, post));


    std::shared_ptr<EvaderPost> future_post;
    std::vector<proto::Unit> future_units;
    std::vector<std::vector<proto::Unit> > future_history;

    if (this->future_steps_ > 0) {
        future_post = post->clone();
        future_units = curr_units;
        future_history = history;
    }

    for (uint32_t i = 0; i < this->future_steps_; ++i) {
        // stay still
        std::for_each(future_units.begin(), future_units.end(),
                [] (proto::Unit & u) {
                    u.set_action(proto::Unit_Action_NOTHING);
                });

        // add to history
        future_history.push_back(future_units);

        // update posterior
        future_post->update(future_units, future_history, tip_history,
                false, true);

        // with look ahead
        features.push_back(this->dist_to_peaks(moved_units, future_post));
    }


    return features;
}


int FuturePeakFeatures::dist_to_peaks(
        const std::vector<proto::Unit> & curr_units,
        const std::shared_ptr<const EvaderPost> & post) {
    const uint32_t num_units(curr_units.size());
    std::vector<uint32_t> locs;
    for (uint32_t i = 0; i < num_units; ++i) {
        locs.push_back(curr_units.at(i).loc());
    }

    const std::vector<uint32_t> peaks(find_peaks(post->probs(),
                    num_units, this->network_));
    const std::vector<uint32_t> assignments(assign_peaks(peaks,
                    locs, this->network_));

    int dist = 0;
    for (uint32_t i = 0; i < num_units; ++i) {
        dist += static_cast<int>(this->network_->dist(locs.at(i),
                        peaks.at(assignments.at(i))));
    }

    return dist;
}


uint32_t FuturePeakFeatures::num_features() const {
    return this->num_features_;
}



} // namespace coopPE
