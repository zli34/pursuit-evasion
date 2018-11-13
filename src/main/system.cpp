#include "system.hpp"
#include <glog/logging.h>

namespace coopPE {


System::System(const uint32_t & num_pursuers,
        const std::shared_ptr<const Network> & network)
    : network_(network), num_pursuers_(num_pursuers),
      pursuer_units_(num_pursuers), pursuer_history_(0),
      time_points_(0) {
}


void System::pursuer_move(const std::vector<proto::Unit_Action> & action) {
    CHECK_EQ(action.size(), this->num_pursuers_);

    for (uint32_t i = 0; i < this->num_pursuers_; ++i) {
        this->pursuer_units_.at(i).set_action(action.at(i));
    }
}


const std::vector<std::vector<proto::Unit> > & System::pursuer_history() const {
    return this->pursuer_history_;
}


const std::vector<proto::Unit> & System::pursuer_units() const {
    return this->pursuer_units_;
}


void System::pursuer_set_locs(const std::vector<uint32_t> & locs) {
    CHECK_EQ(locs.size(), this->num_pursuers_);
    for (uint32_t i = 0; i < this->num_pursuers_; ++i) {
        this->pursuer_units_.at(i).set_loc(locs.at(i));
    }
}


void System::evader_move(const proto::Unit_Action & action) {
    this->evader_unit_.set_action(action);
}


const std::vector<proto::Unit> & System::evader_history() const {
    return this->evader_history_;
}


const proto::Unit & System::evader_unit() const {
    return this->evader_unit_;
}


void System::evader_set_loc(const uint32_t & loc) {
    this->evader_unit_.set_loc(loc);
}


void System::informant_set_tip(const InformantTip & tip) {
    this->tip_history_.emplace(this->time_points_, tip);
}


const std::map<uint32_t, InformantTip> & System::tip_history() const {
    return this->tip_history_;
}


void System::move_unit(proto::Unit & unit,
        const std::shared_ptr<const Network> & network) {
    const uint32_t curr_loc = unit.loc();
    const proto::Node & curr_node = network->node(curr_loc);
    switch (unit.action()) {
    case proto::Unit_Action_UP: {
        unit.set_loc(curr_node.up());
        break;
    }
    case proto::Unit_Action_DOWN: {
        unit.set_loc(curr_node.down());
        break;
    }
    case proto::Unit_Action_LEFT: {
        unit.set_loc(curr_node.left());
        break;
    }
    case proto::Unit_Action_RIGHT: {
        unit.set_loc(curr_node.right());
        break;
    }
    case proto::Unit_Action_NOTHING: {
        unit.set_loc(curr_node.index());
        break;
    }
    default:
        LOG(FATAL) << "Cannot handle action of type "
                   << Unit_Action_Name(unit.action());
        break;
    }

    // clear action after moving
    unit.clear_action();
}


void System::move_all_units() {
    ++this->time_points_;

    // update history
    this->evader_history_.push_back(this->evader_unit_);
    this->pursuer_history_.push_back(this->pursuer_units_);

    // move units
    this->move_unit(this->evader_unit_, this->network_);
    std::for_each(this->pursuer_units_.begin(), this->pursuer_units_.end(),
            [this](proto::Unit & unit) {
                this->move_unit(unit, this->network_);
            });
}


bool System::caught() const {
    return std::any_of(this->pursuer_units_.begin(), this->pursuer_units_.end(),
            [this](const proto::Unit & unit) {
                const uint32_t dist = this->network_->dist(unit.loc(),
                        this->evader_unit_.loc());
                return dist <= 1;
            });
}


uint32_t System::time_points() const {
    return this->time_points_;
}


void System::reset() {
    std::for_each(this->pursuer_units_.begin(),
            this->pursuer_units_.end(),
            [] (proto::Unit & unit) {
                unit.clear_action();
            });
    this->pursuer_history_.clear();

    this->evader_unit_.clear_action();
    this->evader_history_.clear();

    this->tip_history_.clear();

    this->time_points_ = 0;
}


} // namespace coopPE
