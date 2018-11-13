#ifndef QUADRANT_INFORMANT_HPP
#define QUADRANT_INFORMANT_HPP

#include "informant.hpp"

namespace coopPE {


class QuadrantInformant : public Informant {
protected:
    uint32_t num_quadrants_;
    std::vector<std::vector<uint32_t> > quadrants_;

public:
    QuadrantInformant(const std::shared_ptr<const Network> & network);

    QuadrantInformant(const QuadrantInformant & other);

    virtual ~QuadrantInformant() override = default;

    virtual const InformantTip tip(const proto::Unit & evader_unit) override;

    virtual void reset() override;
};


} // namespace coopPE


#endif // QUADRANT_INFORMANT_HPP
