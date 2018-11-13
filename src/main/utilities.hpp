#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "coopPE.pb.h"
#include "network.hpp"

namespace coopPE {


bool next_move(std::vector<proto::Unit_Action> & moves);

std::vector<uint32_t> find_peaks(const std::vector<double> & weights,
        const uint32_t & num_peaks,
        const std::shared_ptr<const Network> & network);

std::vector<uint32_t> assign_peaks(const std::vector<uint32_t> & peaks,
        const std::vector<uint32_t> & locs,
        const std::shared_ptr<const Network> & network);


} // namespace coopPE


#endif // UTILITIES_HPP
