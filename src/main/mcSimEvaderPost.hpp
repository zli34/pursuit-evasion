#ifndef MC_SIM_EVADER_POST_HPP
#define MC_SIM_EVADER_POST_HPP


#include "evaderPost.hpp"
#include "rwEvader.hpp"

namespace coopPE {


class McSimEvaderPost : public EvaderPost {
protected:
    // number of monte carlo replications
    const uint32_t num_reps_;

    // vector of probabilities
    std::vector<double> probs_;

    // the final location for each simulated trajectory
    uint32_t num_points_;
    std::vector<proto::Unit> sim_units_;
    std::vector<std::vector<proto::Unit> > sim_history_;
    std::vector<RwEvader> sim_agents_;

    // data to construct RwEvader objects
    const std::vector<uint32_t> starting_loc_choices_;
    const std::vector<uint32_t> goal_choices_;
    const std::vector<double> drift_choices_;

public:
    McSimEvaderPost(const std::shared_ptr<const Network> & network,
            const uint32_t & num_reps,
            const std::vector<uint32_t> & starting_loc_choices,
            const std::vector<uint32_t> & goal_choices,
            const std::vector<double> & drift_choices);

    McSimEvaderPost(const McSimEvaderPost & other);

    virtual ~McSimEvaderPost() override = default;

    std::shared_ptr<EvaderPost> clone() const override;

    virtual double prob(const uint32_t & loc) const override;
    virtual double prob(const uint32_t & loc,
            const uint32_t & index) const override;

    virtual std::vector<double> probs() const override;
    virtual std::vector<double> probs(const uint32_t & index) const override;

    virtual void update(
            const std::vector<proto::Unit> & pursuer_units,
            const std::vector<std::vector<proto::Unit> > & pursuer_history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const bool & allow_collisions,
            const bool & allow_goal_state) override;

    virtual bool check_do_over(
            const proto::Unit & curr_unit,
            const RwEvader & evader,
            const std::vector<proto::Unit> & pursuer_units,
            const InformantTip & tip,
            const bool & allow_collisions,
            const bool & allow_goal_state);

    virtual std::shared_ptr<Evader> draw_evader() const override;

    virtual uint32_t sample_behavior() const override;

    virtual void reset() override;

    virtual void checkpoint() override;
    virtual void pop_checkpoint() override;
    virtual void revert_checkpoint() override;
    virtual uint32_t num_checkpoints() const override;

};


} // namespace coopPE


#endif // MC_SIM_EVADER_POST_HPP
