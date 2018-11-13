#ifndef RW_EVADER_HPP
#define RW_EVADER_HPP

#include "evader.hpp"

namespace coopPE {


class RwEvader : public Evader {
protected:
    const std::vector<double> drift_choices_;
    double drift_;

    proto::Unit_Action drift_move(const proto::Unit & curr_unit,
            const std::vector<proto::Unit> & history);

    proto::Unit_Action random_move(const proto::Unit & curr_unit,
            const std::vector<proto::Unit> & history);

public:
    RwEvader(const std::shared_ptr<const Network> & network,
            const std::vector<uint32_t> & starting_loc_choices,
            const std::vector<uint32_t> & goal_choices,
            const std::vector<double> & drift_choices);

    RwEvader(const RwEvader & other);

    ~RwEvader() override = default;

    std::shared_ptr<Evader> clone() const override;

    proto::Unit_Action decide_move(const proto::Unit & curr_unit,
            const std::vector<proto::Unit> & history) override;

    void reset() override;
};


} // namespace coopPE


#endif // RW_EVADER_HPP
