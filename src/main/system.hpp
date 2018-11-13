#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <memory>
#include "network.hpp"
#include "coopPE.pb.h"
#include "informant.hpp"

namespace coopPE {

class System {
private:
    const std::shared_ptr<const Network> network_;

    const uint32_t num_pursuers_;
    std::vector<proto::Unit> pursuer_units_;
    std::vector<std::vector<proto::Unit> > pursuer_history_;

    proto::Unit evader_unit_;
    std::vector<proto::Unit> evader_history_;

    std::map<uint32_t, InformantTip> tip_history_;

    uint32_t time_points_;


public:
    System(const uint32_t & num_pursuers,
            const std::shared_ptr<const Network> & network);

    System(const System & other) = delete;

    void pursuer_move(const std::vector<proto::Unit_Action> & actions);
    const std::vector<std::vector<proto::Unit> > & pursuer_history() const;
    const std::vector<proto::Unit> & pursuer_units() const;
    void pursuer_set_locs(const std::vector<uint32_t> & locs);

    void evader_move(const proto::Unit_Action & action);
    const std::vector<proto::Unit> & evader_history() const;
    const proto::Unit & evader_unit() const;
    void evader_set_loc(const uint32_t & loc);

    void informant_set_tip(const InformantTip & tip);
    const std::map<uint32_t, InformantTip> & tip_history() const;

    static void move_unit(proto::Unit & unit,
            const std::shared_ptr<const Network> & network);

    void move_all_units();

    bool caught() const;

    uint32_t time_points() const;

    void reset();
};

} // namespace coopPE

#endif // SYSTEM_HPP
