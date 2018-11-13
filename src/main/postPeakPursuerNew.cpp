#include "postPeakPursuerNew.hpp"

#include <queue>

#include <algorithm>

#include <glog/logging.h>

namespace coopPE {


PostPeakPursuer::PostPeakPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices,
        const std::shared_ptr<EvaderPost> & evader_post)
    : Pursuer(num_units, network, starting_loc_choices),
        evader_post_(evader_post) {
    this->evader_post_->rng(this->rng());
}


PostPeakPursuer::PostPeakPursuer(const PostPeakPursuer & other)
    : Pursuer(other), evader_post_(other.evader_post_->clone()) {
    this->evader_post_->rng(this->rng());
}


std::shared_ptr<Pursuer> PostPeakPursuer::clone() const {
    return std::shared_ptr<Pursuer>(new PostPeakPursuer(*this));
}


std::vector<proto::Unit_Action> PostPeakPursuer::decide_moves(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    // update posterior distribution
    this->evader_post_->update(curr_units, history, tip_history, false, false);

    // find peaks in posterior
	uint32_t b = this->evader_post_->sample_behavior();
    std::vector<double> probs(this->evader_post_->probs(b));
    std::vector<uint32_t> locs;
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        std::priority_queue<std::pair<double, uint32_t> > loc_ranks;
        for (uint32_t j = 0; j < this->network_->size(); ++j) {
            const proto::Node & node(this->network_->node(j));

            const double rank(probs.at(node.index())
                    + probs.at(node.up())
                    + probs.at(node.down())
                    + probs.at(node.left())
                    + probs.at(node.right()));

            loc_ranks.emplace(rank, j);
        }

        const uint32_t peak_loc(loc_ranks.top().second);
        const proto::Node & peak_node(this->network_->node(peak_loc));
        probs.at(peak_node.index()) = 0.0;
        probs.at(peak_node.up()) = 0.0;
        probs.at(peak_node.down()) = 0.0;
        probs.at(peak_node.left()) = 0.0;
        probs.at(peak_node.right()) = 0.0;

        locs.push_back(peak_loc);
    }

    CHECK_EQ(locs.size(), this->num_units_);

    // sort to use permutation
    std::sort(locs.begin(), locs.end());

    // get best permutation
    std::vector<uint32_t> best_assignment;
    double best_distance = std::numeric_limits<double>::infinity();
    do {
        double distance = 0.0;
        for (uint32_t i = 0; i < this->num_units_; ++i) {
            distance += this->network_->dist(curr_units.at(i).loc(),
                    locs.at(i));
        }

        if (distance < best_distance) {
            best_distance = distance;
            best_assignment = locs;
        }
    } while(std::next_permutation(locs.begin(), locs.end()));

    // find action to get closest to target destination
    std::vector<proto::Unit_Action> actions;
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        const proto::Node & node(this->network_->node(curr_units.at(i).loc()));

        std::priority_queue<
            std::pair<int, std::pair<uint32_t, proto::Unit_Action> > > action_ranks;
        action_ranks.emplace(
                -static_cast<int>(this->network_->dist(
                                node.index(), best_assignment.at(i))),
                std::make_pair(node.index(), proto::Unit_Action_NOTHING));
        action_ranks.emplace(
                -static_cast<int>(this->network_->dist(
                                node.up(), best_assignment.at(i))),
                std::make_pair(node.up(), proto::Unit_Action_UP));
        action_ranks.emplace(
                -static_cast<int>(this->network_->dist(
                                node.down(), best_assignment.at(i))),
                std::make_pair(node.down(), proto::Unit_Action_DOWN));
        action_ranks.emplace(
                -static_cast<int>(this->network_->dist(
                                node.left(), best_assignment.at(i))),
                std::make_pair(node.left(), proto::Unit_Action_LEFT));
        action_ranks.emplace(
                -static_cast<int>(this->network_->dist(
                                node.right(), best_assignment.at(i))),
                std::make_pair(node.right(), proto::Unit_Action_RIGHT));

        // break ties by spreading out units as much as possible
        std::priority_queue<std::pair<int, proto::Unit_Action> > tie_breaker;
        double priority;
        do {
            uint32_t dist_to_units = 0.0;
            const uint32_t next_loc(action_ranks.top().second.first);
            for (uint32_t j = 0; j < this->num_units_; j++) {
                if (j != i) {
                    dist_to_units +=
                        this->network_->dist(next_loc, curr_units.at(j).loc());
                }
            }

            tie_breaker.emplace(dist_to_units,
                    action_ranks.top().second.second);

            priority = action_ranks.top().first;
            action_ranks.pop();
        } while(action_ranks.top().first == priority);

        actions.push_back(tie_breaker.top().second);
    }

    return actions;
}


void PostPeakPursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


void PostPeakPursuer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
}


} // namespace coopPE
