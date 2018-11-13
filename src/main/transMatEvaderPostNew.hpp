#ifndef TRANS_MAT_EVADER_POST_HPP
#define TRANS_MAT_EVADER_POST_HPP

#include "evaderPost.hpp"

#include <list>

#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace coopPE {

using namespace boost::numeric;

class TransMatEvaderPost : public EvaderPost {
protected:
    // vector of probabilities
    std::vector<double> probs_;

	// vector of probabilities of p_{t,b}
	std::vector<double> probs_new_;

    uint32_t num_points_;

    // data to construct RwEvader objects
    const std::vector<uint32_t> starting_loc_choices_;
    const std::vector<uint32_t> goal_choices_;
    const std::vector<double> drift_choices_;

    // probability of going from column to row
    const uint32_t num_behaviors_;
    const std::vector<ublas::compressed_matrix<double, ublas::column_major> >
    behavior_trans_mats_;
    std::vector<ublas::vector<double> > behavior_probs_;
	std::vector<ublas::vector<double> > behavior_probs_new_;
	std::vector<uint32_t<double> > behavior_probs_joint_;
	std::vector<uint32_t<double> > behavior_probs_post_;


    std::vector<double> behavior_tots_drifts_;
    std::vector<ublas::vector<double> > behavior_probs_drifts_;

    // checkpoint
    uint32_t cp_count_;
    std::list<std::vector<double> > cp_probs_;
    std::list<uint32_t> cp_num_points_;
    std::list<std::vector<ublas::vector<double> > > cp_behavior_probs_;
    std::list<std::vector<double> > cp_behavior_tots_drifts_;
    std::list<std::vector<ublas::vector<double> > > cp_behavior_probs_drifts_;

public:
    TransMatEvaderPost(const std::shared_ptr<const Network> & network,
            const std::vector<uint32_t> & starting_loc_choices,
            const std::vector<uint32_t> & goal_choices,
            const std::vector<double> & drift_choices);

    TransMatEvaderPost(const TransMatEvaderPost & other);

    virtual ~TransMatEvaderPost() override = default;

    std::shared_ptr<EvaderPost> clone() const override;

    virtual double prob(const uint32_t & loc) const override;
    virtual double prob(const uint32_t & loc,
            const uint32_t & index) const override;

    virtual std::vector<double> probs() const override;
    virtual std::vector<double> probs(const uint32_t & index) const override;

	virtual std::vector<double> probs_new(const uint32_t & index) const override;

    virtual std::vector<double> probs_post() const override;
    

    virtual void update(
            const std::vector<proto::Unit> & pursuer_units,
            const std::vector<std::vector<proto::Unit> > & pursuer_history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const bool & allow_collisions,
            const bool & allow_goal_state) override;

    virtual std::shared_ptr<Evader> draw_evader() const override;

    virtual void reset() override;

    virtual void checkpoint() override;
    virtual void pop_checkpoint() override;
    virtual void revert_checkpoint() override;
    virtual uint32_t num_checkpoints() const override;

    virtual uint32_t sample_behavior() const override;

    std::vector<ublas::compressed_matrix<double, ublas::column_major> >
    trans_mats() const;
};


} // namespace coopPE


#endif // TRANS_MAT_EVADER_POST_HPP
