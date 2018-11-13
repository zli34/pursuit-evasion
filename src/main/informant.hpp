#ifndef INFORMANT_HPP
#define INFORMANT_HPP

#include "network.hpp"

#include <njm_cpp/tools/random.hpp>

#include <glog/logging.h>

namespace coopPE {

class InformantTip {
private:
    const bool has_tip_;
    const std::vector<uint32_t> locs_;
    const double reliable_;
    const double deceitful_;
    const double noisy_;

public:
    InformantTip();

    InformantTip(const std::vector<uint32_t> & locs,
            const double & reliable,
            const double & deceitful,
            const double & noisy);

    InformantTip(const InformantTip & other);

    inline const bool & has_tip() const {
        return this->has_tip_;
    }

    inline const std::vector<uint32_t> & locs() const {
        CHECK(this->has_tip_);
        return this->locs_;
    }

    inline const double & reliable() const {
        CHECK(this->has_tip_);
        return this->reliable_;
    }

    inline const double & deceitful() const {
        CHECK(this->has_tip_);
        return this->deceitful_;
    }

    inline const double & noisy() const {
        CHECK(this->has_tip_);
        return this->noisy_;
    }
};


class Informant : public njm::tools::RngClass {
protected:
    const std::shared_ptr<const Network> network_;

public:
    Informant(const std::shared_ptr<const Network> & network);

    Informant(const Informant & other);

    virtual ~Informant() = default;

    virtual const InformantTip tip(const proto::Unit & evader_unit) = 0;

    virtual void reset() = 0;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // INFORMANT_HPP
