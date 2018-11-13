#include "vfnMaxPursuer.hpp"

#include "featMaxPursuer.hpp"

#include "runner.hpp"

#include "nullInformantTimer.hpp"

#include "coopPE.pb.h"

#include <njm_cpp/optim/simPerturb.hpp>
#include <njm_cpp/tools/random.hpp>
#include <njm_cpp/linalg/stdVectorAlgebra.hpp>

#include <glog/logging.h>

namespace coopPE {

VfnMaxPursuer::VfnMaxPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices,
        const std::shared_ptr<EvaderPost> & evader_post,
        const std::shared_ptr<Features> & features,
        const uint32_t & num_reps,
        const uint32_t & final_t,
        const double & c,
        const double & t,
        const double & a,
        const double & b,
        const double & ell,
        const double & min_step_size)
    : Pursuer(num_units, network, starting_loc_choices),
      evader_post_(evader_post), features_(features),
      num_reps_(num_reps), final_t_(final_t), c_(c), t_(t), a_(a), b_(b),
      ell_(ell), min_step_size_(min_step_size) {
    this->evader_post_->rng(this->rng());
    this->features_->rng(this->rng());
}


VfnMaxPursuer::VfnMaxPursuer(const VfnMaxPursuer & other)
    : Pursuer(other), evader_post_(other.evader_post_->clone()),
      features_(other.features_->clone()),
      num_reps_(other.num_reps_), final_t_(other.final_t_), c_(other.c_),
      t_(other.t_), a_(other.a_), b_(other.b_), ell_(other.ell_),
      min_step_size_(other.min_step_size_) {
    this->evader_post_->rng(this->rng());
}


std::shared_ptr<Pursuer> VfnMaxPursuer::clone() const {
    return std::shared_ptr<Pursuer>(new VfnMaxPursuer(*this));
}


std::vector<proto::Unit_Action> VfnMaxPursuer::decide_moves(
        const std::vector<proto::Unit> &curr_units,
        const std::vector<std::vector<proto::Unit> > &history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    this->evader_post_->update(curr_units, history, tip_history, false, false);

    // start pursuer from current location
    std::vector<std::vector<uint32_t> > checkpoint_starting_locs;
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        checkpoint_starting_locs.push_back({curr_units.at(i).loc()});
    }


    // set checkpoint to estimate posterior given the current state
    // and not reset to ground zero
    std::shared_ptr<EvaderPost> checkpoint_evader_post(
            this->evader_post_->clone());
    checkpoint_evader_post->rng(this->rng());
    checkpoint_evader_post->checkpoint();

    CHECK_GT(this->final_t_, history.size());
    const uint32_t num_points_to_sim(this->final_t_ - history.size());

    auto f = [&] (const std::vector<double> & par,
            const std::vector<double> & par_orig) -> double {
                 FeatMaxPursuer p(this->num_units_,
                         this->network_, checkpoint_starting_locs,
                         checkpoint_evader_post, this->features_, par);
                 p.rng(this->rng());

                 double value = 0;

                 std::shared_ptr<Evader> e;
                 NullInformantTimer informant_timer;
                 for (uint32_t i = 0; i < this->num_reps_; ++i) {
                     e = this->evader_post_->draw_evader();
                     e->rng(this->rng());

                     System s(p.num_units(), this->network_);

                     const proto::Outcome outcome(runner(&s, &p, e.get(),
                                     &informant_timer,
                                     num_points_to_sim));

                     // this is a minimization algorithm, add outcomes
                     // appropriately
                     if (outcome == proto::Outcome::GOAL) {
                         // bad
                         value += 1.0;
                     } else if (outcome == proto::Outcome::CAUGHT) {
                         // good
                         value -= 1.0;
                     } else {
                         CHECK_EQ(outcome, proto::Outcome::TIME);
                         // slightly bad
                         value += 0.1;
                     }
                 }

                 return value / this->num_reps_;
             };

    std::vector<double> optim_coef(this->features_->num_features(), 0.0);

    njm::optim::SimPerturb sp(f, optim_coef, this->c_, this->t_,
            this->a_, this->b_, this->ell_, this->min_step_size_);
    sp.rng(this->rng());

    njm::optim::ErrorCode ec;
    do {
        ec = sp.step();
    } while (ec == njm::optim::ErrorCode::CONTINUE);

    CHECK_EQ(ec, njm::optim::ErrorCode::SUCCESS);

    // assign coefficients
    optim_coef = sp.par();

    const std::vector<std::vector<proto::Unit_Action> > arg_max_moves(
            this->features_->arg_max(
                    curr_units, history, tip_history, this->evader_post_,
                    optim_coef, njm::linalg::dot_a_and_b));

    CHECK_GT(arg_max_moves.size(), 0);
    if (arg_max_moves.size() > 1) {
        return arg_max_moves.at(this->rng()->rint(0, arg_max_moves.size()));
    } else {
        return arg_max_moves.at(0);
    }
}


void VfnMaxPursuer::reset() {
    this->Pursuer::reset();
    this->evader_post_->reset();
}


void VfnMaxPursuer::rng(const std::shared_ptr<njm::tools::Rng> &rng) {
    this->RngClass::rng(rng);
    this->evader_post_->rng(rng);
    this->features_->rng(rng);
}




} // namespace coopPE
