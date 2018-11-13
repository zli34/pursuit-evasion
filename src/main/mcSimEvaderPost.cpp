#include "mcSimEvaderPost.hpp"

#include "system.hpp"

#include <glog/logging.h>


namespace coopPE {


McSimEvaderPost::McSimEvaderPost(
        const std::shared_ptr<const Network> & network,
        const uint32_t & num_reps,
        const std::vector<uint32_t> & starting_loc_choices,
        const std::vector<uint32_t> & goal_choices,
        const std::vector<double> & drift_choices)
    : EvaderPost(network), num_reps_(num_reps),
      starting_loc_choices_(starting_loc_choices), goal_choices_(goal_choices),
      drift_choices_(drift_choices) {
}


McSimEvaderPost::McSimEvaderPost(const McSimEvaderPost & other)
    : EvaderPost(other), num_reps_(other.num_reps_), probs_(other.probs_),
      sim_units_(other.sim_units_), sim_agents_(other.sim_agents_),
      starting_loc_choices_(other.starting_loc_choices_),
      goal_choices_(other.goal_choices_), drift_choices_(other.drift_choices_) {
}


std::shared_ptr<EvaderPost> McSimEvaderPost::clone() const {
    return std::shared_ptr<EvaderPost>(new McSimEvaderPost(*this));
}


double McSimEvaderPost::prob(const uint32_t & loc) const {
    CHECK_LT(loc, this->network_->size());
    return this->probs_.at(loc);
}


double McSimEvaderPost::prob(const uint32_t & loc,
        const uint32_t & index) const {
    LOG(FATAL) << "NOT IMPLEMENTED";
}


std::vector<double> McSimEvaderPost::probs() const {
    return this->probs_;
}


std::vector<double> McSimEvaderPost::probs(const uint32_t & index) const {
    LOG(FATAL) << "NOT IMPLEMENTED";
}


void McSimEvaderPost::update(
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

    std::fill(this->probs_.begin(), this->probs_.end(), 0.0);

    for (uint32_t i = 0; i < this->num_reps_; ++i) {
        bool do_over = true;
        uint32_t final_loc = this->network_->size();
        uint32_t tries = 0;
        while (do_over) {
            // assume no do_over
            do_over = false;

            RwEvader & evader(this->sim_agents_.at(i));
            proto::Unit & unit(this->sim_units_.at(i));

            // get current number of points
            const uint32_t curr_points = this->sim_history_.at(i).size();

            // simulate
            for (uint32_t j = curr_points; j < this->num_points_; ++j) {
                // check if do_over
                if (this->check_do_over(unit, evader, pursuer_history.at(j),
                                tip_history.find(j) != tip_history.end() ?
                                tip_history.at(j) : InformantTip(),
                                allow_collisions,
                                allow_goal_state)) {
                    do_over = true;
                    break;
                }

                // set action
                unit.set_action(evader.decide_move(unit,
                                this->sim_history_.at(i)));

                // store history
                this->sim_history_.at(i).push_back(unit);

                // move
                System::move_unit(unit, this->network_);
                unit.clear_action();

            }

            // save final location
            final_loc = unit.loc();

            // check if do_over
            if (do_over || this->check_do_over(unit, evader,
                            pursuer_units,
                            tip_history.find(this->num_points_)
                            != tip_history.end() ?
                            tip_history.at(this->num_points_) : InformantTip(),
                            allow_collisions,
                            allow_goal_state)) {
                if (++tries >= 10000) {
                    throw std::runtime_error("Do-over too many times");
                }

                // wipe history for rep i
                this->sim_history_.at(i).clear();

                // reset evader and unit
                evader.reset();
                unit.set_loc(evader.starting_loc());
                unit.clear_action();

                do_over = true;
                continue;
            }
        }

        // increment probability
        this->probs_.at(final_loc) += 1.0 / this->num_reps_;
    }

    // update records
    this->prob_history_.push_back(this->probs_);
    ++this->num_points_;
}


bool McSimEvaderPost::check_do_over(const proto::Unit &curr_unit,
        const RwEvader &evader,
        const std::vector<proto::Unit> &pursuer_units,
        const InformantTip &tip,
        const bool &allow_collisions,
        const bool &allow_goal_state) {
    const double draw(this->rng()->runif_01());
    const bool is_reliable(tip.has_tip() && draw < tip.reliable());
    const bool is_deceitful(tip.has_tip()
            && draw < (tip.reliable() + tip.deceitful())
            && !is_reliable);
    const bool is_noisy(tip.has_tip() && !is_reliable && !is_deceitful);
    if (!allow_collisions && std::any_of(pursuer_units.begin(),
                    pursuer_units.end(),
                    [&curr_unit,this](const proto::Unit & p_unit) {
                        return this->network_->dist(p_unit.loc(),
                                curr_unit.loc()) <= 1;
                    })) {
        return true;
    } else if (!allow_goal_state && evader.at_goal(curr_unit)) {
        return true;
    } else if (is_reliable && std::find(tip.locs().begin(), tip.locs().end(),
                    curr_unit.loc()) == tip.locs().end()) {
        return true;
    } else if (is_deceitful && std::find(tip.locs().begin(), tip.locs().end(),
                    curr_unit.loc()) != tip.locs().end()) {
        return true;
    } else {
        return false;
    }
}


std::shared_ptr<Evader> McSimEvaderPost::draw_evader() const {
    LOG(FATAL) << "This needs implementing";
    return std::shared_ptr<Evader>();
}


uint32_t McSimEvaderPost::sample_behavior() const {
    LOG(FATAL) << "This needs implementing";
}


void McSimEvaderPost::reset() {
    this->EvaderPost::reset();
    this->num_points_ = 0;
    // reserve so copy constructor is not called
    this->sim_agents_.clear();
    this->sim_agents_.reserve(this->num_reps_);
    this->sim_units_.clear();
    this->sim_units_.reserve(this->num_reps_);
    this->sim_history_.clear();
    this->sim_history_.resize(this->num_reps_);

    this->probs_.resize(this->network_->size());
    std::fill(this->probs_.begin(), this->probs_.end(), 0.);

    for (uint32_t i = 0; i < this->num_reps_; ++i) {
        this->sim_agents_.emplace_back(this->network_,
                this->starting_loc_choices_, this->goal_choices_,
                this->drift_choices_);
        RwEvader & new_evader(this->sim_agents_.at(i));
        new_evader.rng(this->rng());
        new_evader.reset();

        this->sim_units_.emplace_back();
    }
}


void McSimEvaderPost::checkpoint() {
    LOG(FATAL) << "Not setup to allow checkpoints";
}

void McSimEvaderPost::pop_checkpoint() {
    LOG(FATAL) << "Not setup to allow checkpoints";
}

void McSimEvaderPost::revert_checkpoint() {
    LOG(FATAL) << "Not setup to allow checkpoints";
}

uint32_t McSimEvaderPost::num_checkpoints() const {
    LOG(FATAL) << "Not setup to allow checkpoints";
}


} // namespace coopPE
