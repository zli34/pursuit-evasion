#include "rwPursuer.hpp"
#include <random>
#include <glog/logging.h>


namespace coopPE {

RwPursuer::RwPursuer(const uint32_t & num_units,
        const std::shared_ptr<const Network> & network,
        const std::vector<std::vector<uint32_t> > & starting_loc_choices)
    : Pursuer(num_units, network, starting_loc_choices) {
}


RwPursuer::RwPursuer(const RwPursuer & other)
    : Pursuer(other) {
}


std::shared_ptr<Pursuer> RwPursuer::clone() const {
    return std::shared_ptr<RwPursuer>(new RwPursuer(*this));
}


std::vector<proto::Unit_Action> RwPursuer::decide_moves(
        const std::vector<proto::Unit> & curr_units,
        const std::vector<std::vector<proto::Unit> > & history,
        const std::map<uint32_t, InformantTip> & tip_history) {
    std::vector<proto::Unit_Action> actions;
    for (uint32_t i = 0; i < this->num_units_; ++i) {
        const proto::Unit_Action choice(static_cast<proto::Unit_Action>(
                        this->rng()->rint(proto::Unit_Action_Action_MIN,
                                proto::Unit_Action_Action_ARRAYSIZE)));
        actions.push_back(choice);
    }

    return actions;
}


} // namespace coopPE
