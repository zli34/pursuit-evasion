#ifndef GRID_WORLD_PURSUER_HPP
#define GRID_WORLD_PURSUER_HPP

#include "pursuer.hpp"

#include "evaderPost.hpp"

namespace coopPE {


class GridWorldPursuer : public Pursuer {
protected:
    const std::shared_ptr<EvaderPost> evader_post_;
    const double gamma_;

public:
    GridWorldPursuer(const uint32_t & num_units,
            const std::shared_ptr<const Network> & network,
            const std::vector<std::vector<uint32_t> > & starting_loc_choices,
            const std::shared_ptr<EvaderPost> & evader_post,
            const double & gamma);

    GridWorldPursuer(const GridWorldPursuer & other);

    ~GridWorldPursuer() override = default;

    std::shared_ptr<Pursuer> clone() const override;

    std::vector<proto::Unit_Action> decide_moves(
            const std::vector<proto::Unit> & curr_units,
            const std::vector<std::vector<proto::Unit> > & history,
            const std::map<uint32_t, InformantTip> & tip_history) override;

    proto::Unit_Action decide_one_move(
            const uint32_t & moving_unit,
            const std::vector<proto::Unit> & curr_units);


    static std::vector<double> calc_vfn(
            const std::vector<double> & rewards,
            const double & gamma,
            const std::shared_ptr<const Network> & network);


    void reset() override;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // GRID_WORLD_PURSUER_HPP
