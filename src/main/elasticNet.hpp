#ifndef ELASTIC_NET_HPP
#define ELASTIC_NET_HPP

#include <vector>

#include <cstdint>

namespace coopPE {


std::vector<double> fit_elastic_net_cv(
        const std::vector<double> & y,
        const std::vector<std::vector<double> > & x,
        const uint32_t & num_obs,
        const uint32_t & num_var,
        const std::vector<double> & alpha_vals,
        const std::vector<double> & lambda_vals);


} // namespace coopPE


#endif // ELASTIC_NET_HPP
