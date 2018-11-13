#ifndef INFORMANT_TIMER_HPP
#define INFORMANT_TIMER_HPP


#include "informant.hpp"
#include <njm_cpp/tools/random.hpp>


namespace coopPE {


class InformantTimer : public njm::tools::RngClass {
public:
    InformantTimer() = default;
    virtual ~InformantTimer() = default;

    virtual InformantTip generate_tip(const proto::Unit & evader_unit) = 0;

    virtual void reset() = 0;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // INFORMANT_TIMER_HPP
