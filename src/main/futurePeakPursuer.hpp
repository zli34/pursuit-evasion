#ifndef FUTURE_PEAK_PURSUER_HPP
#define FUTURE_PEAK_PURSUER_HPP

#include "pursuer.hpp"
#include "evaderPost.hpp"

namespace coopPE {


class FuturePeakPursuer : public Pursuer {
protected:
    const std::shared_ptr<EvaderPost> evader_post_;
    const uint32_t future_steps_;

public:
    FuturePeakPursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_loc_choices,
            const std::shared_ptr<EvaderPost> & evader_post,
            const uint32_t & future_steps);

    FuturePeakPursuer(const FuturePeakPursuer & other);

    virtual ~FuturePeakPursuer() override = default;

    std::shared_ptr<Pursuer> clone() const override;

    std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) override;

    double future_peak_dist(
            std::vector<proto::Unit> & curr_units,
            std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<EvaderPost> & post,
            const uint32_t & steps_left);

    void reset() override;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // FUTURE_PEAK_PURSUER_HPP
