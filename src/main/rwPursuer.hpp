#ifndef RW_PURSUER_HPP
#define RW_PURSUER_HPP


#include "coopPE.pb.h"
#include "pursuer.hpp"

namespace coopPE {

class RwPursuer : public Pursuer {
private:

public:
    RwPursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_loc_choices);

    RwPursuer(const RwPursuer & other);

    ~RwPursuer() override = default;

    std::shared_ptr<Pursuer> clone() const override;

    std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) override;

};

} // namespace coopPE


#endif // _PLAYER_HPP
