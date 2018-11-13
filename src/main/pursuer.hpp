#ifndef PURSUER_HPP
#define PURSUER_HPP

#include "coopPE.pb.h"
#include "network.hpp"
#include "informant.hpp"
#include <njm_cpp/tools/random.hpp>

namespace coopPE {


class Pursuer : public njm::tools::RngClass {
protected:
    const uint32_t num_units_;
    const std::shared_ptr<const Network> network_;
    const std::vector<std::vector<uint32_t> > starting_locs_choices_;

    std::vector<uint32_t> starting_locs_;

public:
    Pursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_locs_choices);

    Pursuer(const Pursuer & other);

    virtual ~Pursuer() = default;

    virtual std::shared_ptr<Pursuer> clone() const = 0;

    virtual std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) = 0;

    virtual std::vector<uint32_t> starting_locs();

    virtual void reset();

    uint32_t num_units() const;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};



} // namespace coopPE


#endif // PURSUER_HPP
