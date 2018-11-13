#ifndef BASIS_LOADING_FEATURES_HPP
#define BASIS_LOADING_FEATURES_HPP

#include "features.hpp"

namespace coopPE {


class BasisLoadingFeatures : public Features {
protected:
    const uint32_t spline_order_;
    const uint32_t max_x_knots_;
    const uint32_t max_y_knots_;

public:
    BasisLoadingFeatures(const std::shared_ptr<const Network> & network,
            const uint32_t & spline_order,
            const uint32_t & max_x_knots,
            const uint32_t & max_y_knots);

    BasisLoadingFeatures(const BasisLoadingFeatures & other);

    virtual ~BasisLoadingFeatures() override = default;

    virtual std::shared_ptr<Features> clone() const override;

    virtual std::vector<double> get_features(
            const std::vector<uint32_t> & locs,
            const std::vector<Unit_Action> & actions,
            const std::function<double(double, double)> & post) override;

    // virtual std::vector<double> update_features(
    //         const std::vector<uint32_t> & locs,
    //         const std::vector<Unit_Action> & actions,
    //         const std::function<double(double, double)> & post) override;

    using njm::tools::RngClass::rng;
    virtual void rng(const std::shared_ptr<njm::tools::Rng> & rng) override;
};


} // namespace coopPE


#endif // BASIS_LOADING_FEATURES_HPP
