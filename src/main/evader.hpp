#ifndef EVADER_HPP
#define EVADER_HPP

#include "coopPE.pb.h"
#include "network.hpp"
#include <njm_cpp/tools/random.hpp>

namespace coopPE {


class Evader : public njm::tools::RngClass {
protected:
    const std::shared_ptr<const Network> network_;

    const std::vector<uint32_t> starting_loc_choices_;
    const std::vector<uint32_t> goal_choices_;

    uint32_t starting_loc_;
    uint32_t goal_;

public:
    Evader(const std::shared_ptr<const Network> & network,
            const std::vector<uint32_t> & starting_loc_choices,
            const std::vector<uint32_t> & goal_choices);

    Evader(const Evader & other);

    virtual ~Evader() = default;

    virtual std::shared_ptr<Evader> clone() const = 0;

    virtual proto::Unit_Action decide_move(const proto::Unit & curr_unit,
            const std::vector<proto::Unit> & history) = 0;

    virtual uint32_t starting_loc();

    virtual void reset();

    bool at_goal(const proto::Unit & curr_unit) const;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE



#endif // EVADER_HPP
