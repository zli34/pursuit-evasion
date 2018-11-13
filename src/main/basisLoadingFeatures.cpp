#include "basisLoadingFeatures.hpp"

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_vector.h>

#include "elasticNet.hpp"


namespace coopPE {


BasisLoadingFeatures::BasisLoadingFeatures(
        const std::shared_ptr<const Network> & network,
        const uint32_t & spline_order,
        const uint32_t & max_x_knots,
        const uint32_t & max_y_knots)
    : Features(network), spline_order_(spline_order), max_x_knots_(max_x_knots),
      max_y_knots_(max_y_knots) {
}


BasisLoadingFeatures::BasisLoadingFeatures(const BasisLoadingFeatures & other)
    : Features(other), spline_order_(other.spline_order_),
      max_x_knots_(other.max_x_knots_), max_y_knots_(other.max_y_knots_) {
}


std::shared_ptr<Features> BasisLoadingFeatures::clone() const {
    return std::shared_ptr<Features>(new BasisLoadingFeatures(*this));
}


std::vector<double> BasisLoadingFeatures::get_features(
        const std::vector<uint32_t> & locs,
        const std::vector<Unit_Action> & actions,
        const std::function<double(double, double)> & post) {
    // setup b spline workspaces
    gsl_bspline_workspace * x_bs_w(
            gsl_bspline_alloc(this->spline_order_, this->max_x_knots_));
    gsl_bspline_knots_uniform(0.0, 1.0, x_bs_w);
    gsl_bspline_workspace * y_bs_w(
            gsl_bspline_alloc(this->spline_order_, this->max_y_knots_));
    gsl_bspline_knots_uniform(0.0, 1.0, y_bs_w);

    // initialize vectors for b spline evaluation
    const uint32_t num_x_vals = this->spline_order_ + this->max_x_knots_ - 2;
    gsl_vector * x_vals(gsl_vector_alloc(num_x_vals));
    const uint32_t num_y_vals = this->spline_order_ + this->max_y_knots_ - 2;
    gsl_vector * y_vals(gsl_vector_alloc(num_y_vals));

    // probabilities
    std::vector<double> probs;
    // basis funcitons
    std::vector<std::vector<double> > basis;
    for (uint32_t i = 0; i < this->network_->size(); ++i) {
        // evaluate at each node in the network
        const Node & node(this->network_->node(i));
        const double prob(post(node.x(), node.y()));

        if (prob > 0.0) {
            probs.push_back(std::log(prob));

            gsl_bspline_eval(node.x(), x_vals, x_bs_w);
            gsl_bspline_eval(node.y(), y_vals, y_bs_w);

            std::vector<double> add_to_basis;
            // intercept
            add_to_basis.push_back(1.0);
            for (uint32_t x_pos = 0; x_pos < num_x_vals; ++x_pos) {
                for (uint32_t y_pos = 0; y_pos < num_y_vals; ++y_pos) {
                    add_to_basis.push_back(gsl_vector_get(x_vals, x_pos)
                            * gsl_vector_get(y_vals, y_pos));
                }
            }
            basis.push_back(std::move(add_to_basis));
        }
    }

    // free gsl containers
    gsl_bspline_free(x_bs_w);
    gsl_bspline_free(y_bs_w);
    gsl_vector_free(x_vals);
    gsl_vector_free(y_vals);

    const std::vector<double> alpha_vals({
                0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0});
    const std::vector<double> lambda_vals({
                1.0, 10.0, 50.0, 100.0, 500.0, 1000.0});
    const std::vector<double> beta_hat(fit_elastic_net_cv(probs, basis,
                    this->network_->size(), 1 + num_x_vals * num_y_vals,
                    alpha_vals, lambda_vals));

    // TODO: now include features with actions
}


void BasisLoadingFeatures::rng(const std::shared_ptr<njm::tools::Rng> & rng) {
    this->RngClass::rng(rng);
}


} // namespace coopPE
