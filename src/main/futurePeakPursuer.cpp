#include "futurePeakPursuer.hpp"

#include "system.hpp"

#include "utilities.hpp"

#include <glog/logging.h>

namespace coopPE {


FuturePeakPursuer::FuturePeakPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices,
        const std::shared_ptr<EvaderPost> & evader_post,
        const uint32_t & future_steps)
    : Pursuer(num_units, network, starting_loc_choices),
      evader_post_(evader_post), future_steps_(future_steps) {
    this->evader_post_->rng(this->rng());
}


FuturePeakPursuer::FuturePeakPursuer(const FuturePeakPursuer & other)
    : Pursuer(other), evader_post_(other.evader_post_->clone()),
      future_steps_(other.future_steps_) {
    this->evader_post_->rng(this->rng());
}


std::shared_ptr<Pursuer> FuturePeakPursuer::clone() const {
    return std::shared_ptr<Pursuer>(new FuturePeakPursuer(*this));
}


std::vector<proto::Unit_Action> FuturePeakPursuer::decide_moves(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    // update evader post
    this->evader_post_->update(curr_units, history, tip_history, false, false);

    std::shared_ptr<EvaderPost> future_post(this->evader_post_->clone());
    std::vector<proto::Unit> future_units(curr_units);
    std::vector<std::vector<proto::Unit> > future_history(history);

    for (uint32_t i = 0; i < this->future_steps_; ++i) {
        // don't move
        std::for_each(future_units.begin(), future_units.end(),
                [] (proto::Unit & u) {
                    u.set_action(proto::Unit_Action_NOTHING);
                });

        // add to history
        future_history.push_back(future_units);

        // move units
        std::for_each(future_units.begin(), future_units.end(),
                [this] (proto::Unit & u) {
                    System::move_unit(u, this->network_);
                });

        // update posterior
        future_post->update(future_units, future_history, tip_history,
                false, true);
    }

    const std::vector<uint32_t> peaks(find_peaks(future_post->probs(),
                    this->num_units_, this->network_));

    std::pair<int, int> best_score(
            std::numeric_limits<int>::lowest(),
            std::numeric_limits<int>::lowest());
    std::vector<std::vector<proto::Unit_Action> > best_actions;
    std::vector<proto::Unit_Action> actions(this->num_units_,
            proto::Unit_Action_NOTHING);

    do {
        std::vector<proto::Unit> moving_units(curr_units);
        std::vector<uint32_t> locs;
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            moving_units.at(i).set_action(actions.at(i));
            System::move_unit(moving_units.at(i), this->network_);
            locs.push_back(moving_units.at(i).loc());
        }

        std::pair<int, int> score(0, 0);
        const std::vector<uint32_t> assignments(assign_peaks(peaks, locs,
                        this->network_));

        for (uint32_t i = 0; i < this->num_units_; ++i) {
            score.first -= static_cast<int>(this->network_->dist(locs.at(i),
                            peaks.at(assignments.at(i))));

            for (uint32_t j = (i + 1); j < this->num_units_; ++j) {
                score.second += static_cast<int>(this->network_->dist(
                                locs.at(i), locs.at(j)));
            }
        }

        if (score > best_score) {
            best_score = score;
            best_actions.clear();
            best_actions.push_back(actions);
        } else if (score == best_score) {
            best_actions.push_back(actions);
        }

    } while(next_move(actions));



    // // record keeping
    // std::vector<std::vector<Unit_Action> > best_actions;
    // double best_dist_to_peaks(std::numeric_limits<double>::infinity());

    // // find best
    // std::vector<Unit_Action> actions(this->num_units_, Unit_Action_NOTHING);
    // do {
    //     // copy because they will be change
    //     std::vector<Unit> moving_units(curr_units);
    //     std::vector<std::vector<Unit> > moving_history(history);
    //     for (uint32_t i = 0; i < this->num_units_; ++i) {
    //         moving_units.at(i).set_action(actions.at(i));
    //     }

    //     const double dist_to_peaks(this->future_peak_dist(moving_units,
    //                     moving_history, this->evader_post_->clone(),
    //                     this->future_steps_));

    //     if (dist_to_peaks < best_dist_to_peaks) {
    //         best_actions.clear();
    //         best_actions.push_back(actions);
    //         best_dist_to_peaks = dist_to_peaks;
    //     } else if(dist_to_peaks == best_dist_to_peaks) {
    //         best_actions.push_back(actions);
    //     }
    // } while(next_move(actions));

    return best_actions.at(this->rng()->rint(0, best_actions.size()));
}


double FuturePeakPursuer::future_peak_dist(
        std::vector<proto::Unit> & curr_units,
        std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const std::shared_ptr<EvaderPost> & post,
        const uint32_t & steps_left) {
    // take step
    history.push_back(curr_units);
    std::for_each(curr_units.begin(), curr_units.end(),
            [this] (proto::Unit & u) {
                System::move_unit(u, this->network_);
            });

    // update posterior
    post->update(curr_units, history, tip_history, false, true);

    // get peaks
    const std::vector<uint32_t> peaks(find_peaks(post->probs(),
                    this->num_units_, this->network_));

    // assign peaks
    std::vector<uint32_t> locs;
    std::for_each(curr_units.begin(), curr_units.end(),
            [&locs] (const proto::Unit & u) {
                locs.push_back(u.loc());
            });
    const std::vector<uint32_t> assignments(assign_peaks(peaks, locs,
                    this->network_));

    if (steps_left == 0) {
        // return dist to peaks
        double dist_to_peaks = 0.0;
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            dist_to_peaks += this->network_->dist(peaks.at(assignments.at(i)),
                    locs.at(i));
        }

        return dist_to_peaks;
    } else {
        // find best next actions and take another step
        std::vector<proto::Unit_Action> actions(this->num_units_,
                proto::Unit_Action_NOTHING);
        std::vector<std::vector<proto::Unit_Action> > best_actions;
        std::pair<int, int> best_rank(
                std::numeric_limits<int>::lowest(), // dist to peaks
                std::numeric_limits<int>::lowest()); // neg dist to others
        do {
            // copy and move units
            std::vector<proto::Unit> moving_units(curr_units);
            for (uint32_t i = 0; i < this->num_units_; ++i) {
                moving_units.at(i).set_action(actions.at(i));
                System::move_unit(moving_units.at(i), this->network_);
            }


            // get distances to peaks and others
            std::pair<int, int> rank(0.0, 0.0);
            for (uint32_t i = 0; i < this->num_units_; i++) {
                // subtract dist to peaks
                rank.first -= static_cast<int>(
                        this->network_->dist(peaks.at(assignments.at(i)),
                                moving_units.at(i).loc()));

                for (uint32_t j = (i + 1); j < this->num_units_; ++j) {
                    // add dist to others
                    rank.second += static_cast<int>(
                            this->network_->dist(
                                    moving_units.at(i).loc(),
                                    moving_units.at(j).loc()));
                }
            }


            if (rank > best_rank) {
                best_actions.clear();
                best_actions.push_back(actions);
                best_rank = rank;
            } else if (rank == best_rank) {
                best_actions.push_back(actions);
            }

        } while (next_move(actions));

        CHECK_GT(best_actions.size(), 0);
        actions = best_actions.at(this->rng()->rint(0, best_actions.size()));
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            curr_units.at(i).set_action(actions.at(i));
        }

        return this->future_peak_dist(curr_units, history, tip_history, post,
                steps_left - 1);
    }
}


void FuturePeakPursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


void FuturePeakPursuer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
}

} // namespace coopPE
