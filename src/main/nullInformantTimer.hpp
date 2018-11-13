#ifndef NULL_INFORMANT_TIMER_HPP
#define NULL_INFORMANT_TIMER_HPP

#include "informantTimer.hpp"

namespace coopPE {


/**
 * Never generate a tip
 */
class NullInformantTimer : public InformantTimer {
public:
    NullInformantTimer() = default;
    virtual ~NullInformantTimer() override = default;

    InformantTip generate_tip(const proto::Unit & evader_unit) override {
        return InformantTip();
    };

    void reset() override {};
};


} // namespace coopPE


#endif // NULL_INFORMANT_TIMER_HPP
