#ifndef BRUTE_FORCE_PURSUER_HPP
#define BRUTE_FORCE_PURSUER_HPP

#include "pursuer.hpp"
#include "features.hpp"

namespace coopPE {


class BruteForcePursuer : public Pursuer {
protected:
    const std::shared_ptr<EvaderPost> evader_post_;
    const std::shared_ptr<Features> features_;
    const std::vector<double> par_;

public:
    BruteForcePursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_loc_choices,
            const std::shared_ptr<EvaderPost> & evader_post,
            const std::shared_ptr<Features> & features,
            const std::vector<double> & par);

    BruteForcePursuer(const BruteForcePursuer & other);

    virtual ~BruteForcePursuer() override = default;

    virtual std::shared_ptr<Pursuer> clone() const override;

    virtual std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) override;

    bool next_move(std::vector<proto::Unit_Action> & moves) const;

    void reset() override;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // BRUTE_FORCE_PURSUER_HPP
