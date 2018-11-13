#include "features.hpp"

#include "utilities.hpp"

#include "system.hpp"


namespace coopPE {


Features::Features(const std::shared_ptr<const Network> & network)
    : network_(network) {
}


Features::Features(const Features & other)
    : network_(other.network_) {
}


void Features::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
}


std::vector<std::vector<proto::Unit_Action> > Features::arg_max(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history,
        const std::shared_ptr<const EvaderPost> & post,
        const std::vector<double> & coef,
        const std::function<double(const std::vector<double> & coef,
                const std::vector<double> & feats)> & fn) {
    const uint32_t num_units(curr_units.size());
    std::vector<proto::Unit> future_units(curr_units);

    std::vector<proto::Unit_Action> actions(num_units,
            proto::Unit_Action_NOTHING);

    std::vector<std::vector<proto::Unit_Action> > best_actions;
    std::pair<double, int> best_score(std::numeric_limits<int>::lowest(),
            std::numeric_limits<int>::lowest());
    do
    {
        // set actions
        for (uint32_t i = 0; i < num_units; ++i) {
            future_units.at(i).set_action(actions.at(i));
        }

        // store moved units in separate container
        std::vector<proto::Unit> moved_units(future_units);
        std::for_each(moved_units.begin(), moved_units.end(),
                [this] (proto::Unit & u) {
                    System::move_unit(u, this->network_);
                });

        // get features and store value
        const std::vector<double> feats(this->get_features(future_units,
                        history, tip_history, post));
        std::pair<double, int> score;
        score.first = fn(coef, feats);

        // calculate distance between units as tie breaker
        score.second = 0;
        for (uint32_t i = 0; i < num_units; ++i) {
            for (uint32_t j = (i + 1); j < num_units; ++j) {
                score.second -= this->network_->dist(moved_units.at(i).loc(),
                        moved_units.at(j).loc());
            }
        }

        if (score > best_score) {
            best_score = score;
            best_actions.clear();
            best_actions.push_back(actions);
        } else if (score == best_score) {
            best_actions.push_back(actions);
        }

    } while (next_move(actions));

    return best_actions;
}



} // namespace coopPE
