#include <gtest/gtest.h>
#include <glog/logging.h>
#include "elasticNet.hpp"

#include <njm_cpp/tools/random.hpp>
#include <njm_cpp/linalg/stdVectorAlgebra.hpp>

namespace coopPE {


TEST(TestElasticNet, CoefRecovery) {
    const uint32_t num_obs(10);
    const uint32_t num_var(50);

    njm::tools::Rng rng;

    std::vector<double> eps(num_obs);
    std::generate(eps.begin(), eps.end(),
            [&rng]() {
                return rng.rnorm_01();
            });

    std::vector<double> beta(num_var);
    std::generate(beta.begin(), beta.end(),
            [&rng]() {
                return rng.rnorm_01();
            });


    std::vector<double> y;
    std::vector<std::vector<double> > x;

    for (uint32_t i = 0; i < num_obs; ++i) {
        std::vector<double> x_i(num_var);
        // intercept value
        x_i.at(0) = 1.0;
        // non-intercept values
        std::generate(x_i.begin() + 1, x_i.end(),
                [&rng]() {
                    return rng.runif_01();
                });

        y.push_back(njm::linalg::dot_a_and_b(x_i, beta) + eps.at(i));
        x.push_back(std::move(x_i));
    }

    const std::vector<double> alpha_vals({
                0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0});
    const std::vector<double> lambda_vals({
                1.0, 10.0, 50.0, 100.0, 500.0, 1000.0});
    const std::vector<double> beta_hat(fit_elastic_net_cv(y, x,
                    num_obs, num_var, alpha_vals, lambda_vals));

    // calculate sum squared relative error
    double error = 0.0;
    for (uint32_t i = 0; i < num_obs; ++i) {
        const double y_hat(njm::linalg::dot_a_and_b(x.at(i), beta_hat));
        error += (y_hat - y.at(i)) * (y_hat - y.at(i)) / y.at(i);
    }
    EXPECT_LT(error / num_obs, 0.1);
}


} // namespace coopPE


int main(int argc, char *argv[]) {
    ::google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
