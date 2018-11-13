#ifndef EVADER_POST_HPP
#define EVADER_POST_HPP

#include "network.hpp"

#include "evader.hpp"

#include "informant.hpp"

#include <njm_cpp/tools/random.hpp>


namespace coopPE {


class EvaderPost : public njm::tools::RngClass {
protected:
    const std::shared_ptr<const Network> network_;

    std::vector<std::vector<double> > prob_history_;

public:
    EvaderPost(const std::shared_ptr<const Network> & network);

    EvaderPost(const EvaderPost & other);

    virtual ~EvaderPost() = default;

    virtual std::shared_ptr<EvaderPost> clone() const = 0;

    virtual double prob(const uint32_t & loc) const = 0;
    virtual double prob(const uint32_t & loc, const uint32_t & index) const = 0;

    virtual std::vector<double> probs() const = 0;
    virtual std::vector<double> probs(const uint32_t & index) const = 0;

    virtual void update(
            const std::vector<proto::Unit> & pursuer_units,
            const std::vector<std::vector<proto::Unit> > & pursuer_history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const bool & allow_collisions,
            const bool & allow_goal_state) = 0;

    virtual std::shared_ptr<Evader> draw_evader() const = 0;

    virtual void reset() = 0;

    virtual void checkpoint() = 0;
    virtual void pop_checkpoint() = 0;
    virtual void revert_checkpoint() = 0;
    virtual uint32_t num_checkpoints() const = 0;
    virtual uint32_t sample_behavior() const = 0;

    virtual std::vector<std::vector<double> > history() const;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE



#endif // EVADER_POST_HPP
