#ifndef EWT_INFORMANT_TIMER_HPP
#define EWT_INFORMANT_TIMER_HPP

#include "informantTimer.hpp"

namespace coopPE {


/**
 * An informant timer using exponential wait times
 */
class EwtInformantTimer : public InformantTimer {
private:
    const double rate_;
    const std::shared_ptr<Informant> informant_;

    uint32_t num_points_;
    double threshold_;

public:
    EwtInformantTimer(const double & rate,
            const std::shared_ptr<Informant> & informant);

    virtual ~EwtInformantTimer() override = default;

    InformantTip generate_tip(const proto::Unit & evader_unit) override;

    void reset() override;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // EWT_INFORMANT_TIMER_HPP
