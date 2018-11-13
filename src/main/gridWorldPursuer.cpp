#include "gridWorldPursuer.hpp"

#include "system.hpp"

#include <njm_cpp/linalg/stdVectorAlgebra.hpp>


namespace coopPE {


GridWorldPursuer::GridWorldPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices,
        const std::shared_ptr<EvaderPost> & evader_post,
        const double & gamma)
    : Pursuer(num_units, network, starting_loc_choices),
      evader_post_(evader_post), gamma_(gamma) {
}


GridWorldPursuer::GridWorldPursuer(const GridWorldPursuer & other)
        : Pursuer(other), evader_post_(other.evader_post_->clone()),
        gamma_(other.gamma_) {
}


std::shared_ptr<Pursuer> GridWorldPursuer::clone() const {
    return std::shared_ptr<Pursuer>(new GridWorldPursuer(*this));
}


std::vector<proto::Unit_Action> GridWorldPursuer::decide_moves(
        const std::vector<proto::Unit> &curr_units,
        const std::vector<std::vector<proto::Unit> > &history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    // update posterior distribution
    this->evader_post_->update(curr_units, history, tip_history, false, false);

    std::vector<proto::Unit_Action> actions;
    std::vector<proto::Unit> next_units(curr_units);
    // Move each unit incrementally.  Each unit chooses the best move
    // conditional on what the previous units have picked thus far.
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        // get move for unit i
        const proto::Unit_Action move = this->decide_one_move(i, next_units);
        next_units.at(i).set_action(move);


        // move unit for use in remaining units
        System::move_unit(next_units.at(i), this->network_);

        // save action
        actions.push_back(move);
    }
    return actions;
}


proto::Unit_Action GridWorldPursuer::decide_one_move(
        const uint32_t & moving_unit,
        const std::vector<proto::Unit> & curr_units) {
    // use probs as reward
    std::vector<double> probs(this->evader_post_->probs());
    // zero out positions of previous units
    for (uint32_t i = 0; i < moving_unit; ++i) {
        const proto::Node & node(this->network_->node(curr_units.at(i).loc()));
        probs.at(node.index()) = 0.0;
        probs.at(node.up()) = 0.0;
        probs.at(node.down()) = 0.0;
        probs.at(node.left()) = 0.0;
        probs.at(node.right()) = 0.0;
    }

    std::vector<double> rewards;
    for (uint32_t i = 0; i < this->network_->size(); ++i) {
        const proto::Node & node(this->network_->node(i));
        std::set<uint32_t> neigh;
        neigh.insert(node.index());
        neigh.insert(node.up());
        neigh.insert(node.down());
        neigh.insert(node.left());
        neigh.insert(node.right());

        rewards.push_back(std::accumulate(neigh.begin(), neigh.end(), 0.0,
                        [&probs] (const double & total,
                                const uint32_t & index) {
                            return total + probs.at(index);
                        }));
    }

    const std::vector<double> vfn(GridWorldPursuer::calc_vfn(rewards,
                    this->gamma_, this->network_));

    std::vector<std::pair<double, proto::Unit_Action> > action_values;
    const proto::Node & node(this->network_->node(curr_units.at(moving_unit).loc()));
    // up
    action_values.emplace_back(vfn.at(node.up()), proto::Unit_Action_UP);
    // down
    action_values.emplace_back(vfn.at(node.down()), proto::Unit_Action_DOWN);
    // left
    action_values.emplace_back(vfn.at(node.left()), proto::Unit_Action_LEFT);
    // right
    action_values.emplace_back(vfn.at(node.right()), proto::Unit_Action_RIGHT);
    // nothing
    action_values.emplace_back(vfn.at(node.index()), proto::Unit_Action_NOTHING);

    const proto::Unit_Action best_action(std::max_element(action_values.begin(),
                    action_values.end())->second);

    return best_action;
}


std::vector<double> GridWorldPursuer::calc_vfn(
        const std::vector<double> & rewards,
        const double & gamma,
        const std::shared_ptr<const Network> & network) {
    bool converged = false;
    std::vector<double> vfn(rewards);
    std::vector<double> last_vfn(rewards);
    while(!converged) {
        // save last values
        last_vfn = vfn;

        double largest_diff = 0.0;
        // update each element of vfn
        for (uint32_t i = 0; i < network->size(); ++i) {
            const proto::Node & node(network->node(i));
            std::vector<std::pair<double, uint32_t> > neigh_vfn;
            // up
            neigh_vfn.emplace_back(
                    rewards.at(node.up()) + gamma * vfn.at(node.up()),
                    node.up());
            // down
            neigh_vfn.emplace_back(
                    rewards.at(node.down()) + gamma * vfn.at(node.down()),
                    node.down());
            // left
            neigh_vfn.emplace_back(
                    rewards.at(node.left()) + gamma * vfn.at(node.left()),
                    node.left());
            // right
            neigh_vfn.emplace_back(
                    rewards.at(node.right()) + gamma * vfn.at(node.right()),
                    node.right());
            // nothing
            neigh_vfn.emplace_back(rewards.at(i) + gamma * vfn.at(i), i);


            // get best neighbor
            const auto best_neigh = *std::max_element(
                    neigh_vfn.begin(), neigh_vfn.end());

            vfn.at(i) = best_neigh.first;

            largest_diff = std::max(largest_diff,
                    std::abs(vfn.at(i) - last_vfn.at(i)));
        }

        // test largest difference for convergence criteria
        converged = largest_diff < 1e-8;
    }
    return vfn;
}


void GridWorldPursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


void GridWorldPursuer::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
}




} // namespace coopPE
