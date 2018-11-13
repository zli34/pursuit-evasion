#ifndef QFN_ROLLOUT_PURSUER_HPP
#define QFN_ROLLOUT_PURSUER_HPP

#include "pursuer.hpp"
#include "evaderPost.hpp"

namespace coopPE {


class QfnRolloutPursuer : public Pursuer {
protected:
    const std::shared_ptr<EvaderPost> evader_post_;
    const uint32_t rollout_steps_;
    const uint32_t vfn_approx_steps_;

public:
    QfnRolloutPursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_locs_choices,
            const std::shared_ptr<EvaderPost> & evader_post,
            const uint32_t & rollout_steps,
            const uint32_t & vfn_approx_steps);

    QfnRolloutPursuer(const QfnRolloutPursuer & other);

    virtual ~QfnRolloutPursuer() override = default;

    std::shared_ptr<Pursuer> clone() const override;

    std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) override;

    void reset() override;

    double vfn_rollout(const double &curr_value,
            const uint32_t & curr_rollout,
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<EvaderPost> & curr_ep,
            const uint32_t & behavior_index);

    double vfn_approx(const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const std::shared_ptr<EvaderPost> & curr_ep,
            const uint32_t & behavior_index);

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // QFN_ROLLOUT_PURSUER_HPP
