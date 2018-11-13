#ifndef RUNNER_HPP
#define RUNNER_HPP

#include "coopPE.pb.h"
#include "system.hpp"
#include "pursuer.hpp"
#include "evader.hpp"
#include "informantTimer.hpp"

namespace coopPE {

proto::Outcome runner(System * s, Pursuer * p, Evader * e, InformantTimer * i,
        const uint32_t & time_points);


} // namespace coopPE


#endif // RUNNER_HPP
