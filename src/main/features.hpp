#ifndef FEATURES_HPP
#define FEATURES_HPP


#include "network.hpp"
#include "evaderPost.hpp"
#include "informant.hpp"

#include <njm_cpp/tools/random.hpp>


namespace coopPE {


class Features : public njm::tools::RngClass {
protected:
    const std::shared_ptr<const Network> network_;

public:
    Features(const std::shared_ptr<const Network> & network);

    Features(const Features & other);

    virtual ~Features() = default;

    virtual std::shared_ptr<Features> clone() const = 0;

    virtual uint32_t num_features() const = 0;

    virtual std::vector<double> get_features(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<const EvaderPost> & post) = 0;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;

    std::vector<std::vector<proto::Unit_Action> > arg_max(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<const EvaderPost> & post,
            const std::vector<double> & coef,
            const std::function<double(const std::vector<double> & coef,
                    const std::vector<double> & feats)> & fn);
};


} // namespace coopPE


#endif // FEATURES_HPP
