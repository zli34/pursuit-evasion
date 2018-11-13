#ifndef VFN_MAX_PURSUER_HPP
#define VFN_MAX_PURSUER_HPP

#include "pursuer.hpp"
#include "evaderPost.hpp"
#include "features.hpp"

namespace coopPE {


class VfnMaxPursuer : public Pursuer {
protected:
    const std::shared_ptr<EvaderPost> evader_post_;
    const std::shared_ptr<Features> features_;

    const uint32_t num_reps_;
    const uint32_t final_t_;
    const double c_;
    const double t_;
    const double a_;
    const double b_;
    const double ell_;
    const double min_step_size_;

public:
    VfnMaxPursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_loc_choices,
            const std::shared_ptr<EvaderPost> & evader_post,
            const std::shared_ptr<Features> & features,
            const uint32_t & num_reps,
            const uint32_t & final_t,
            const double & c,
            const double & t,
            const double & a,
            const double & b,
            const double & ell,
            const double & min_step_size);

    VfnMaxPursuer(const VfnMaxPursuer & other);

    ~VfnMaxPursuer() override = default;

    std::shared_ptr<Pursuer> clone() const override;

    std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) override;

    void reset() override;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // VFN_MAX_PURSUER_HPP
