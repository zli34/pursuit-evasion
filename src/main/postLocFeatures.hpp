#ifndef POST_LOC_FEATURES_HPP
#define POST_LOC_FEATURES_HPP

#include "features.hpp"

namespace coopPE {


class PostLocFeatures : public Features {
protected:
    const uint32_t num_pursuers_;
    const uint32_t num_features_;
    double dist_scale_;

public:
    PostLocFeatures(const std::shared_ptr<const Network> & network,
            const uint32_t & num_pursuers);

    PostLocFeatures(const PostLocFeatures & other);

    virtual ~PostLocFeatures() = default;

    virtual std::shared_ptr<Features> clone() const override;

    virtual uint32_t num_features() const override;

    virtual std::vector<double> get_features(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<const EvaderPost> & post) override;
};


} // namespace coopPE


#endif // POST_LOC_FEATURES_HPP
