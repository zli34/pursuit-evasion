#include "qfnRolloutPursuer.hpp"
#include "system.hpp"
#include "utilities.hpp"

namespace coopPE {


QfnRolloutPursuer::QfnRolloutPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_locs_choices,
        const std::shared_ptr<EvaderPost> & evader_post,
        const uint32_t & rollout_steps,
        const uint32_t & vfn_approx_steps)
    : Pursuer(num_units, network, starting_locs_choices),
      evader_post_(evader_post), rollout_steps_(rollout_steps),
      vfn_approx_steps_(vfn_approx_steps) {
    this->evader_post_->rng(this->rng());
}


QfnRolloutPursuer::QfnRolloutPursuer(const QfnRolloutPursuer & other)
    : Pursuer(other), evader_post_(other.evader_post_->clone()),
      rollout_steps_(other.rollout_steps_),
      vfn_approx_steps_(other.vfn_approx_steps_) {
}


std::shared_ptr<Pursuer> QfnRolloutPursuer::clone() const {
    return std::shared_ptr<Pursuer>(new QfnRolloutPursuer(*this));
}


std::vector<proto::Unit_Action> QfnRolloutPursuer::decide_moves(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history) {

    this->evader_post_->update(curr_units, history, tip_history, false, false);
    this->evader_post_->checkpoint();

    const uint32_t behavior_index(this->evader_post_->sample_behavior());

    double best_value(-std::numeric_limits<double>::infinity());
    std::vector<proto::Unit_Action> best_actions;
    std::vector<proto::Unit_Action> actions(this->num_units_,
            proto::Unit_Action_Action_MIN);
    do {
        if (std::all_of(actions.begin(), actions.end(),
                        [](const proto::Unit_Action & a_) {
                            return a_ != proto::Unit_Action_NOTHING;
                        })) {
            std::vector<proto::Unit> next_units(curr_units);
            std::vector<std::vector<proto::Unit> > next_history(history);

            // set actions
            for (uint32_t i = 0; i < this->num_units_; ++i) {
                next_units.at(i).set_action(actions.at(i));
            }
            next_history.push_back(next_units);

            // move units
            for (uint32_t i = 0; i < this->num_units_; ++i) {
                System::move_unit(next_units.at(i), this->network_);
            }

            // update posterior for reward
            this->evader_post_->revert_checkpoint();
            this->evader_post_->update(next_units, next_history,
                    tip_history, true, false);

            const std::vector<double> probs(
                    this->evader_post_->probs(behavior_index));

            // update reward
            double next_value(0.0);
            std::set<uint32_t> caught_locs;
            for (uint32_t i = 0; i < this->num_units_; ++i) {
                const proto::Unit & u(next_units.at(i));
                const proto::Node & n(this->network_->node(u.loc()));

                if (caught_locs.count(n.index()) == 0) {
                    caught_locs.insert(n.index());
                    next_value += probs.at(n.index());
                }

                if (caught_locs.count(n.up()) == 0) {
                    caught_locs.insert(n.up());
                    next_value += probs.at(n.up());
                }

                if (caught_locs.count(n.down()) == 0) {
                    caught_locs.insert(n.down());
                    next_value += probs.at(n.down());
                }

                if (caught_locs.count(n.left()) == 0) {
                    caught_locs.insert(n.left());
                    next_value += probs.at(n.left());
                }

                if (caught_locs.count(n.right()) == 0) {
                    caught_locs.insert(n.right());
                    next_value += probs.at(n.right());
                }
            }

            // update posterior for reward
            this->evader_post_->revert_checkpoint();
            this->evader_post_->update(next_units, next_history,
                    tip_history, false, false);

            // get reward for future
            const double rollout_value(this->vfn_rollout(next_value,
                            0, next_units, next_history,
                            tip_history, this->evader_post_,
                            behavior_index));

            // check if future rollout is non-trivial
            if (next_value < 1.0 && rollout_value > 0.0) {
                next_value += std::exp(std::log(1.0 - next_value)
                        + std::log(rollout_value));
            }

            // check if better than the previous
            if (best_value < next_value) {
                best_value = next_value;
                best_actions = actions;
            }
        }

        // increment actions
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            if (actions.at(i) != proto::Unit_Action_Action_MAX) {
                actions.at(i) = static_cast<proto::Unit_Action>(
                        actions.at(i) + 1);
                break;
            } else {
                actions.at(i) = proto::Unit_Action_Action_MIN;
            }
        }

    } while(!std::all_of(actions.begin(), actions.end(),
                    [] (const proto::Unit_Action & a_) {
                        return a_ == proto::Unit_Action_Action_MIN;
                    }));

    // pop checkpoint
    this->evader_post_->pop_checkpoint();

    CHECK_EQ(this->evader_post_->num_checkpoints(), 0);

    return best_actions;
}


void QfnRolloutPursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


double QfnRolloutPursuer::vfn_rollout(const double & curr_value,
        const uint32_t & curr_rollout,
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const std::shared_ptr<EvaderPost> & curr_ep,
        const uint32_t & behavior_index) {
    if (curr_rollout == this->rollout_steps_) {
        curr_ep->checkpoint();
        // roll out value using heuristic policy
        const double approx(this->vfn_approx(curr_units, history, tip_history,
                        curr_ep, behavior_index));
        curr_ep->pop_checkpoint();

        CHECK_EQ(this->evader_post_->num_checkpoints(), curr_rollout + 1);

        return approx;
    } else {
        curr_ep->checkpoint();

        std::vector<proto::Unit_Action> actions(this->num_units_,
                proto::Unit_Action_Action_MIN);

        double best_value(std::numeric_limits<double>::lowest());
        do {
            if (std::all_of(actions.begin(), actions.end(),
                            [](const proto::Unit_Action & a_) {
                                return a_ != proto::Unit_Action_NOTHING;
                            })) {
                std::vector<proto::Unit> next_units(curr_units);
                std::vector<std::vector<proto::Unit> > next_history(history);

                // set actions
                for (uint32_t i = 0; i < this->num_units_; ++i) {
                    next_units.at(i).set_action(actions.at(i));
                }
                next_history.push_back(next_units);

                // move units
                for (uint32_t i = 0; i < this->num_units_; ++i) {
                    System::move_unit(next_units.at(i), this->network_);
                }

                // update posterior for reward
                curr_ep->revert_checkpoint();
                curr_ep->update(next_units, next_history, tip_history,
                        true, false);

                // update reward
                double next_value(0.0);
                std::set<uint32_t> caught_locs;
                for (uint32_t i = 0; i < this->num_units_; ++i) {
                    const proto::Unit & u(next_units.at(i));
                    const proto::Node & n(this->network_->node(u.loc()));

                    if (caught_locs.count(n.index()) == 0) {
                        caught_locs.insert(n.index());
                        next_value += curr_ep->prob(n.index(),
                                behavior_index);
                    }

                    if (caught_locs.count(n.up()) == 0) {
                        caught_locs.insert(n.up());
                        next_value += curr_ep->prob(n.up(),
                                behavior_index);
                    }

                    if (caught_locs.count(n.down()) == 0) {
                        caught_locs.insert(n.down());
                        next_value += curr_ep->prob(n.down(),
                                behavior_index);
                    }

                    if (caught_locs.count(n.left()) == 0) {
                        caught_locs.insert(n.left());
                        next_value += curr_ep->prob(n.left(),
                                behavior_index);
                    }

                    if (caught_locs.count(n.right()) == 0) {
                        caught_locs.insert(n.right());
                        next_value += curr_ep->prob(n.right(),
                                behavior_index);
                    }
                }

                // update posterior for no catch
                curr_ep->revert_checkpoint();
                curr_ep->update(next_units, next_history, tip_history,
                        false, false);

                // get reward for future
                const double rollout_value(this->vfn_rollout(next_value,
                                curr_rollout + 1, next_units, next_history,
                                tip_history, curr_ep, behavior_index));

                // check if future rollout is non-trivial
                if (next_value < 1.0 && rollout_value > 0.0) {
                    next_value += std::exp(std::log(1.0 - next_value)
                            + std::log(rollout_value));
                }

                // check if better than the previous
                best_value = std::max(best_value, next_value);
            }

            // increment actions
            for (uint32_t i = 0; i < this->num_units_; ++i) {
                if (actions.at(i) != proto::Unit_Action_Action_MAX) {
                    actions.at(i) = static_cast<proto::Unit_Action>(
                            actions.at(i) + 1);
                    break;
                } else {
                    actions.at(i) = proto::Unit_Action_Action_MIN;
                }
            }

        } while(!std::all_of(actions.begin(), actions.end(),
                        [] (const proto::Unit_Action & a_) {
                            return a_ == proto::Unit_Action_Action_MIN;
                        }));

        // pop checkpoint
        curr_ep->pop_checkpoint();

        CHECK_EQ(this->evader_post_->num_checkpoints(), curr_rollout + 1);

        return best_value;
    }
}


double QfnRolloutPursuer::vfn_approx(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const std::shared_ptr<EvaderPost> & curr_ep,
        const uint32_t & behavior_index) {
    std::vector<double> rewards;
    std::vector<proto::Unit> next_units(curr_units);
    std::vector<std::vector<proto::Unit> > next_history(history);

    for (uint32_t i = 0; i < this->vfn_approx_steps_; i++) {
        // get posterior if no pursuers move
        std::vector<proto::Unit> still_units(next_units);
        std::for_each(still_units.begin(), still_units.end(),
                [] (proto::Unit & u_) {
                    u_.set_action(proto::Unit_Action_NOTHING);
                });

        std::vector<std::vector<proto::Unit> > still_history(next_history);
        still_history.push_back(still_units);

        // get posterior
        curr_ep->checkpoint();
        curr_ep->update(still_units, still_history, tip_history, true, false);
        const std::vector<double> probs(curr_ep->probs(behavior_index));

        // update reward
        double next_value(0.0);
        std::set<uint32_t> caught_locs;
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            const proto::Unit & u(next_units.at(i));
            const proto::Node & n(this->network_->node(u.loc()));

            if (caught_locs.count(n.index()) == 0) {
                caught_locs.insert(n.index());
                next_value += probs.at(n.index());
            }

            if (caught_locs.count(n.up()) == 0) {
                caught_locs.insert(n.up());
                next_value += probs.at(n.up());
            }

            if (caught_locs.count(n.down()) == 0) {
                caught_locs.insert(n.down());
                next_value += probs.at(n.down());
            }

            if (caught_locs.count(n.left()) == 0) {
                caught_locs.insert(n.left());
                next_value += probs.at(n.left());
            }

            if (caught_locs.count(n.right()) == 0) {
                caught_locs.insert(n.right());
                next_value += probs.at(n.right());
            }
        }
        rewards.push_back(next_value);

        // find peaks
        const std::vector<uint32_t> peaks(find_peaks(probs,
                        this->num_units_, this->network_));

        std::vector<uint32_t> locs(this->num_units_);
        std::transform(still_units.begin(), still_units.end(), locs.begin(),
                [] (const proto::Unit & u_) {return u_.loc();});

        // peak assignments
        const std::vector<uint32_t> assignments(assign_peaks(peaks,
                        locs, this->network_));

        for (uint32_t j = 0; j < this->num_units_; ++j) {
            std::vector<proto::Unit_Action> best_actions;
            double best_dist(std::numeric_limits<double>::infinity());

            const uint32_t & peak_j(peaks.at(assignments.at(j)));

            for (uint32_t k = proto::Unit_Action_Action_MIN;
                 k <= proto::Unit_Action_Action_MAX; ++k) {
                proto::Unit u;
                u.set_loc(next_units.at(j).loc());
                u.set_action(static_cast<proto::Unit_Action>(k));
                System::move_unit(u, this->network_);
                const double dist(this->network_->dist(u.loc(), peak_j));

                if (dist < best_dist) {
                    best_actions.clear();
                    best_actions.push_back(static_cast<proto::Unit_Action>(k));
                } else if (dist == best_dist) {
                    best_actions.push_back(static_cast<proto::Unit_Action>(k));
                }
            }

            CHECK_GT(best_actions.size(), 0);
            if (best_actions.size() > 1) {
                const uint32_t action_ind(
                        this->rng_->rint(0, best_actions.size()));
                next_units.at(j).set_action(best_actions.at(action_ind));
            } else {
                next_units.at(j).set_action(best_actions.at(0));
            }
        }

        next_history.push_back(next_units);
        std::for_each(next_units.begin(), next_units.end(),
                [this] (proto::Unit & u_) {
                    System::move_unit(u_, this->network_);
                });

        curr_ep->pop_checkpoint();
        curr_ep->update(next_units, next_history, tip_history, false, false);


    }

    CHECK_EQ(this->vfn_approx_steps_, rewards.size());

    double total(0.0);
    std::reverse(rewards.begin(), rewards.end());
    for (uint32_t i = 1; i < this->vfn_approx_steps_; i++) {
        if (total > 0.0 && rewards.at(i) < 1.0) {
            total = std::exp(std::log(total)
                    + std::log(1.0 - rewards.at(i)))
                + rewards.at(i);
        } else {
            total = rewards.at(i);
        }
    }

    return total;
}


void QfnRolloutPursuer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
}

} // namespace coopPE
