#include "transMatEvaderPost.hpp"

#include "rwEvader.hpp"

#include <boost/numeric/ublas/operation.hpp>
#include <queue>

#include <njm_cpp/linalg/stdVectorAlgebra.hpp>

#include <glog/logging.h>

namespace coopPE {


TransMatEvaderPost::TransMatEvaderPost(
        const std::shared_ptr<const Network> & network,
        const std::vector<uint32_t> & starting_loc_choices,
        const std::vector<uint32_t> & goal_choices,
        const std::vector<double> & drift_choices)
    : EvaderPost(network), probs_(), num_points_(0),
      starting_loc_choices_(starting_loc_choices),
      goal_choices_(goal_choices), drift_choices_(drift_choices),
      num_behaviors_(this->goal_choices_.size() * this->drift_choices_.size()),
      behavior_trans_mats_(this->trans_mats()),
      behavior_probs_(this->num_behaviors_,
              ublas::vector<double>(this->network_->size())),
      cp_count_(0) {
}


TransMatEvaderPost::TransMatEvaderPost(const TransMatEvaderPost & other)
    : EvaderPost(other), probs_(other.probs_), num_points_(other.num_points_),
      starting_loc_choices_(other.starting_loc_choices_),
      goal_choices_(other.goal_choices_), drift_choices_(other.drift_choices_),
      num_behaviors_(other.num_behaviors_),
      behavior_trans_mats_(other.behavior_trans_mats_),
      behavior_probs_(other.behavior_probs_),
      cp_count_(other.cp_count_), cp_probs_(other.cp_probs_),
      cp_num_points_(other.cp_num_points_),
      cp_behavior_probs_(other.cp_behavior_probs_),
      cp_behavior_tots_drifts_(other.cp_behavior_tots_drifts_),
      cp_behavior_probs_drifts_(other.cp_behavior_probs_drifts_) {

}


std::shared_ptr<EvaderPost> TransMatEvaderPost::clone() const {
    return std::shared_ptr<EvaderPost>(new TransMatEvaderPost(*this));
}


double TransMatEvaderPost::prob(const uint32_t & loc) const {
    CHECK_LT(loc, this->network_->size());
    return this->probs_.at(loc);
}


double TransMatEvaderPost::prob(const uint32_t & loc,
        const uint32_t & index) const {
    CHECK_LT(loc, this->network_->size());
    return this->behavior_probs_drifts_.at(index)[loc];
}


std::vector<double> TransMatEvaderPost::probs() const {
    CHECK_EQ(this->probs_.size(), this->network_->size());
    return this->probs_;
}


std::vector<double> TransMatEvaderPost::probs(const uint32_t & index) const {
    CHECK_EQ(this->probs_.size(), this->network_->size());
    const ublas::vector<double> & probs_index(
            this->behavior_probs_drifts_.at(index));
    std::vector<double> probs_vec(this->network_->size());
    probs_vec.assign(probs_index.begin(), probs_index.end());
    return probs_vec;
}

uint32_t TransMatEvaderPost::sample_behavior() const {
    double tot_prob(0.0);
    const double draw(this->rng()->runif_01());
    uint32_t last_good_index(this->drift_choices_.size());
    for (uint32_t i = 0; i < this->drift_choices_.size(); ++i) {
        const double prob_to_add(this->behavior_tots_drifts_.at(i));
        if (prob_to_add > 0.0) {
            last_good_index = i;
        }

        tot_prob += prob_to_add;
        if (tot_prob > draw) {
            return i;
        }
    }
    // only happens with rounding errors and a draw close to 1.0
    CHECK_LT(last_good_index, this->num_behaviors_);
    return last_good_index;
}


void TransMatEvaderPost::update(
        const std::vector<proto::Unit> & pursuer_units,
        const std::vector<std::vector<proto::Unit> > & pursuer_history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const bool & allow_collisions,
        const bool & allow_goal_state) {
    CHECK_EQ(pursuer_history.size(), this->num_points_)
        << "history is size " << pursuer_history.size()
        << " and number of points is " << this->num_points_;

    const uint32_t num_pursuers = pursuer_units.size();
    std::for_each(pursuer_history.begin(), pursuer_history.end(),
            [&num_pursuers](const std::vector<proto::Unit> & v) {
                CHECK_EQ(v.size(), num_pursuers);
            });

    if (this->num_points_ == 0) {
        // uniform over each starting location
        for (uint32_t b = 0; b < this->num_behaviors_; ++b) {
            std::fill(this->behavior_probs_.at(b).begin(),
                    this->behavior_probs_.at(b).end(), 0.0);

            std::for_each(this->starting_loc_choices_.begin(),
                    this->starting_loc_choices_.end(),
                    [this, &b] (const uint32_t & starting_loc) {
                        this->behavior_probs_.at(b)(starting_loc) +=
                            1.0 / this->starting_loc_choices_.size();
                    });
        }
    } else {
        // turn one transition from previous location probabilities
        for (uint32_t b = 0; b < this->num_behaviors_; ++b) {
            ublas::axpy_prod(this->behavior_trans_mats_.at(b),
                    ublas::vector<double>(this->behavior_probs_.at(b)),
                    this->behavior_probs_.at(b), true);
        }
    }

    // zero out locations with purusers and adjust for informants
    double sum_total_prob(0.0);
    for (uint32_t b = 0; b < this->num_behaviors_; ++b) {
        // adjust for pursuer locations
        std::for_each(pursuer_units.begin(), pursuer_units.end(),
                [this, &b, &allow_goal_state, &allow_collisions]
                (const proto::Unit & p_unit) {
                    const proto::Node & node(this->network_->node(
                                    p_unit.loc()));
                    if (!allow_collisions) {
                        this->behavior_probs_.at(b)(node.index()) = 0.0;
                        this->behavior_probs_.at(b)(node.up()) = 0.0;
                        this->behavior_probs_.at(b)(node.down()) = 0.0;
                        this->behavior_probs_.at(b)(node.left()) = 0.0;
                        this->behavior_probs_.at(b)(node.right()) = 0.0;
                    }

                    const uint32_t num_goals(this->goal_choices_.size());
                    if (!allow_goal_state) {
                        for (uint32_t i = 0; i < num_goals; ++i) {
                            this->behavior_probs_.at(b)(
                                    this->goal_choices_.at(i)) = 0.0;
                        }
                    }
                });

        // adjust for informants
        if (tip_history.find(this->num_points_) != tip_history.end()
                && tip_history.at(this->num_points_).has_tip()) {
            const InformantTip & tip(tip_history.at(this->num_points_));
            CHECK(tip.has_tip());

            const std::vector<uint32_t> & locs(tip.locs());
            // informant situations
            ublas::vector<double> reliable(this->network_->size(), 0.0);
            ublas::vector<double> deceitful(
                    tip.deceitful() * this->behavior_probs_.at(b));
            for (auto it = locs.begin(); it != locs.end(); ++it) {
                reliable(*it) =
                    tip.reliable() * this->behavior_probs_.at(b)(*it);

                deceitful(*it) = 0.0;
            }

            // scale for noisy, then add in reliable and deceitful
            this->behavior_probs_.at(b) *= tip.noisy();
            this->behavior_probs_.at(b) += reliable + deceitful;
        }

        sum_total_prob += ublas::sum(this->behavior_probs_.at(b));
    }

    if (sum_total_prob == 0.0) {
        throw std::runtime_error("Pursuers are covering all areas");
    }

    // scale probabilities
    std::for_each(this->behavior_probs_.begin(),
            this->behavior_probs_.end(),
            [&sum_total_prob] (ublas::vector<double> & v) {
                v /= sum_total_prob;
            });

    // aggregate over goals to get proability conditional on drift
    this->behavior_probs_drifts_.resize(this->drift_choices_.size());
    for (uint32_t d = 0; d < this->drift_choices_.size(); ++d) {
        this->behavior_probs_drifts_.at(d).resize(this->network_->size());
        this->behavior_probs_drifts_.at(d) *= 0.0;
    }
    for (uint32_t g = 0, i = 0; g < this->goal_choices_.size(); ++g) {
        for (uint32_t d = 0; d < this->drift_choices_.size(); ++d, ++i) {
            this->behavior_probs_drifts_.at(d) += this->behavior_probs_.at(i);
        }
    }
    this->behavior_tots_drifts_.resize(this->drift_choices_.size());
    for (uint32_t d = 0; d < this->drift_choices_.size(); ++d) {
        const double sum(ublas::sum(this->behavior_probs_drifts_.at(d)));
        this->behavior_tots_drifts_.at(d) = sum;
        if (sum > 0.0) {
            this->behavior_probs_drifts_.at(d) /= sum;
        }
    }

    // assign probabilities
    const ublas::vector<double> tot_prob(std::accumulate(
                    this->behavior_probs_.begin(),
                    this->behavior_probs_.end(),
                    ublas::vector<double>(this->network_->size(), 0.0)));
    CHECK_EQ(tot_prob.size(), this->network_->size());
    this->probs_.assign(tot_prob.begin(), tot_prob.end());

    CHECK_NEAR(ublas::sum(tot_prob), 1.0, 1e-5);

    // update records
    this->prob_history_.push_back(this->probs_);
    ++this->num_points_;

    CHECK_EQ(this->prob_history_.size(), this->num_points_);
}


std::shared_ptr<Evader> TransMatEvaderPost::draw_evader() const {
    const double draw(this->rng()->runif_01());
    double total = 0.0;
    uint32_t goal_draw(this->goal_choices_.size());
    uint32_t drift_draw(this->drift_choices_.size());
    uint32_t loc_draw(this->network_->size());
    bool done = false;
    for (uint32_t g = 0, b = 0; g < this->goal_choices_.size(); ++g) {
        for (uint32_t d = 0; d < this->drift_choices_.size(); ++d, ++b) {
            for (uint32_t i = 0; i < this->network_->size(); ++i) {
                // increment total
                total += this->behavior_probs_.at(b)[i];

                if (total > draw) {
                    goal_draw = g;
                    drift_draw = d;
                    loc_draw = i;
                    done = true;
                    break;
                }
            }
            if (done) {
                break;
            }
        }
        if (done) {
            break;
        }
    }

    if (!done) {
        goal_draw = this->goal_choices_.size() - 1;
        drift_draw = this->drift_choices_.size() - 1;
        loc_draw = this->network_->size() - 1;
    }

    return std::shared_ptr<Evader>(
            new RwEvader(this->network_,
                    {loc_draw},
                    {this->goal_choices_.at(goal_draw)},
                    {this->drift_choices_.at(drift_draw)}));
}


void TransMatEvaderPost::reset() {
    this->EvaderPost::reset();
    this->num_points_ = 0;

    this->probs_.resize(this->network_->size());

    this->behavior_probs_.resize(this->num_behaviors_);
    for (uint32_t b = 0; b < this->num_behaviors_; ++b) {
        std::fill(this->behavior_probs_.at(b).begin(),
                this->behavior_probs_.at(b).end(), 0.0);
    }

    this->cp_count_ = 0;
    this->cp_probs_.clear();
    this->cp_num_points_.clear();
    this->cp_behavior_probs_.clear();
    this->cp_behavior_tots_drifts_.clear();
    this->cp_behavior_probs_drifts_.clear();
}


void TransMatEvaderPost::checkpoint() {
    ++this->cp_count_;
    this->cp_probs_.push_back(this->probs_);
    this->cp_num_points_.push_back(this->num_points_);
    this->cp_behavior_probs_.push_back(this->behavior_probs_);
    this->cp_behavior_tots_drifts_.push_back(this->behavior_tots_drifts_);
    this->cp_behavior_probs_drifts_.push_back(this->behavior_probs_drifts_);
}


void TransMatEvaderPost::pop_checkpoint() {
    CHECK_GT(this->cp_count_, 0);
    --this->cp_count_;
    this->probs_ = std::move(this->cp_probs_.back());
    this->cp_probs_.pop_back();

    this->num_points_ = std::move(this->cp_num_points_.back());
    this->cp_num_points_.pop_back();

    this->behavior_probs_ = std::move(this->cp_behavior_probs_.back());
    this->cp_behavior_probs_.pop_back();

    this->prob_history_.erase(this->prob_history_.begin() + this->num_points_,
            this->prob_history_.end());

    this->behavior_tots_drifts_ =
        std::move(this->cp_behavior_tots_drifts_.back());
    this->cp_behavior_tots_drifts_.pop_back();

    this->behavior_probs_drifts_ =
        std::move(this->cp_behavior_probs_drifts_.back());
    this->cp_behavior_probs_drifts_.pop_back();
}


void TransMatEvaderPost::revert_checkpoint() {
    CHECK_GT(this->cp_count_, 0);
    this->probs_ = this->cp_probs_.back();

    this->num_points_ = this->cp_num_points_.back();

    this->behavior_probs_ = this->cp_behavior_probs_.back();

    this->prob_history_.erase(this->prob_history_.begin() + this->num_points_,
            this->prob_history_.end());

    this->behavior_tots_drifts_ = this->cp_behavior_tots_drifts_.back();

    this->behavior_probs_drifts_ = this->cp_behavior_probs_drifts_.back();
}


uint32_t TransMatEvaderPost::num_checkpoints() const {
    return this->cp_count_;
}


std::vector<ublas::compressed_matrix<double, ublas::column_major> >
TransMatEvaderPost::trans_mats() const {
    std::vector<ublas::compressed_matrix<double, ublas::column_major> > mats;

    for (uint32_t g = 0; g < this->goal_choices_.size(); ++g) {
        const uint32_t & goal(this->goal_choices_.at(g));

        for (uint32_t d = 0; d < this->drift_choices_.size(); ++d) {
            const double & drift(this->drift_choices_.at(d));

            ublas::compressed_matrix<double, ublas::column_major> m(
                    this->network_->size(), this->network_->size(),
                    this->network_->size() * 5);

            for (uint32_t x = 0; x < this->network_->size(); ++x) {
                const proto::Node & node(this->network_->node(x));

                if (this->network_->dist(x, goal) == 0) {
                    // absorb into goal
                    m(goal, x) += 1.0;
                } else {
                    // probability of random direction
                    m(node.index(), x) += 0.2 * (1.0 - drift);
                    m(node.up(), x) += 0.2 * (1.0 - drift);
                    m(node.down(), x) += 0.2 * (1.0 - drift);
                    m(node.left(), x) += 0.2 * (1.0 - drift);
                    m(node.right(), x) += 0.2 * (1.0 - drift);

                    // get neighbor closest to goal
                    std::priority_queue<std::pair<int, uint32_t> > closest;
                    closest.emplace(-static_cast<int>(
                                    this->network_->dist(node.index(), goal)),
                            node.index());
                    closest.emplace(-static_cast<int>(
                                    this->network_->dist(node.up(), goal)),
                            node.up());
                    closest.emplace(-static_cast<int>(
                                    this->network_->dist(node.down(), goal)),
                            node.down());
                    closest.emplace(-static_cast<int>(
                                    this->network_->dist(node.left(), goal)),
                            node.left());
                    closest.emplace(-static_cast<int>(
                                    this->network_->dist(node.right(), goal)),
                            node.right());

                    // get all of equal distance to goal
                    const int best_dist = closest.top().first;
                    std::vector<uint32_t> all_best;
                    do {
                        all_best.push_back(closest.top().second);
                        closest.pop();
                    } while (closest.top().first == best_dist);

                    // probability if drifting toward goal
                    std::for_each(all_best.begin(), all_best.end(),
                            [&m, &x, &all_best, &drift]
                            (const uint32_t & index) {
                                m(index, x) += drift / all_best.size();
                            });
                }
            }

            mats.push_back(std::move(m));
        }
    }

    return mats;
}


} // namespace coopPE
