#ifndef FUTURE_PEAK_FEATURES_HPP
#define FUTURE_PEAK_FEATURES_HPP

#include "features.hpp"

namespace coopPE {


class FuturePeakFeatures : public Features {
protected:
    const uint32_t future_steps_;
    const uint32_t num_features_;

public:
    FuturePeakFeatures(const std::shared_ptr<const Network> & network,
            const uint32_t & future_steps);

    FuturePeakFeatures(const FuturePeakFeatures & other);

    virtual ~FuturePeakFeatures() override = default;

    virtual std::shared_ptr<Features> clone() const override;

    virtual std::vector<double> get_features(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<const EvaderPost> & post) override;

    int dist_to_peaks(
            const std::vector<proto::Unit> & curr_units,
            const std::shared_ptr<const EvaderPost> & post);

    virtual uint32_t num_features() const override;

};


} // namespace coopPE


#endif // POST_PEAK_FEATURES_HPP
