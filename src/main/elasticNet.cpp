#include "elasticNet.hpp"

#include <mlpack/methods/lars/lars.hpp>

#include <armadillo>

#include <glog/logging.h>

namespace coopPE {


std::vector<double> fit_elastic_net_cv(
        const std::vector<double> & y,
        const std::vector<std::vector<double> > & x,
        const uint32_t & num_obs,
        const uint32_t & num_var,
        const std::vector<double> & alpha_vals,
        const std::vector<double> & lambda_vals) {
    CHECK_EQ(num_obs, y.size());
    CHECK_EQ(num_obs, x.size());

    // create train and test sets
    std::vector<arma::mat> y_train;
    std::vector<arma::mat> x_train;

    std::vector<double> y_test;
    std::vector<arma::rowvec> x_test;

    for (uint32_t i = 0; i < num_obs; ++i) {
        CHECK_EQ(x.at(i).size(), num_var)
            << "observation " << i << " has " << x.at(i).size() << " variables "
            << "but user specified " << num_var << " variables";

        // create test set
        {
            y_test.push_back(y.at(i));
            x_test.push_back(arma::conv_to<arma::rowvec>::from(x.at(i)));
        }

        // create train set
        {
            arma::mat y_mat(num_obs - 1, 1);
            arma::mat x_mat(num_obs - 1, num_var);
            uint32_t row_index = 0;
            for (uint32_t j = 0; j < num_obs; ++j) {
                if (j != i) {
                    y_mat.row(row_index) = y.at(j);
                    x_mat.row(row_index) =
                        arma::conv_to<arma::rowvec>::from(x.at(j));

                    ++row_index;
                }
            }

            y_train.push_back(std::move(y_mat));
            x_train.push_back(std::move(x_mat));
        }
    }

    // cross validation
    const uint32_t num_alpha = alpha_vals.size();
    const uint32_t num_lambda = lambda_vals.size();
    CHECK_GT(num_alpha, 0);
    CHECK_GT(num_lambda, 0);

    std::vector<double>::const_iterator alpha, alpha_end, lambda, lambda_end;
    alpha_end = alpha_vals.end();
    lambda_end = lambda_vals.end();
    double best_error = std::numeric_limits<double>::infinity();
    double best_alpha = -1.0;
    double best_lambda = -1.0;
    for (alpha = alpha_vals.begin(); alpha != alpha_end; ++alpha) {
        for (lambda = lambda_vals.begin(); lambda != lambda_end; ++lambda) {

            mlpack::regression::LARS lars(false, (*lambda) * (*alpha),
                    (*lambda) * (1.0 - (*alpha)));
            double error = 0.0;
            for (uint32_t i = 0; i < num_obs; ++i) {
                arma::vec beta;
                lars.Train(x_train.at(i), y_train.at(i), beta, false);

                const double y_hat = arma::dot(x_test.at(i), beta);
                error += (y_hat - y_test.at(i)) * (y_hat - y_test.at(i));
            }

            if (error < best_error) {
                best_error = error;
                best_alpha = *alpha;
                best_lambda = *lambda;
            }
        }
    }

    // [0, 1]
    CHECK_GE(best_alpha, 0.0);
    CHECK_LE(best_alpha, 1.0);
    // (0, infinity)
    CHECK_GT(best_lambda, 0.0);

    // fit using all data
    arma::mat y_all(num_obs, 1);
    arma::mat x_all(num_obs, num_var);
    for (uint32_t i = 0; i < num_obs; ++i) {
        y_all(i) = y.at(i);
        x_all.row(i) = arma::conv_to<arma::rowvec>::from(x.at(i));
    }

    mlpack::regression::LARS lars(false, best_lambda * best_alpha,
            best_lambda * (1.0 - best_alpha));
    arma::vec beta;
    lars.Train(x_all, y_all, beta, false);


    return arma::conv_to<std::vector<double> >::from(beta);
}


} // namespace coopPE
