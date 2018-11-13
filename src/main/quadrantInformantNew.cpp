#include "quadrantInformantNew.hpp"
#include<iostream>
#include<cstdlib>
#include<ctime>
#define random(a,b) (rand()%(b-a+1)+a)

namespace coopPE {


QuadrantInformant::QuadrantInformant(
        const std::shared_ptr<const Network> & network)
    : Informant(network) {
    CHECK_EQ(this->network_->init().type(), proto::NetworkInit_NetType_GRID);

    const uint32_t dim_x(this->network_->init().dim_x());
    const uint32_t dim_y(this->network_->init().dim_y());

    const uint32_t dim_x_mid(dim_x / 2);
    const uint32_t dim_y_mid(dim_y / 2);

    // top_left, bottom_left, top_right, bottom_right
    this->quadrants_.clear();
    this->quadrants_.resize(4);
    for (uint32_t x = 0, i = 0; x < dim_x; ++x) {
        for (uint32_t y = 0; y < dim_y; ++y, ++i) {
            if (x < dim_x_mid && y < dim_y_mid) {
                this->quadrants_.at(0).push_back(i);
            } else if(x < dim_x_mid) {
                this->quadrants_.at(1).push_back(i);
            } else if(y < dim_y_mid) {
                this->quadrants_.at(2).push_back(i);
            } else {
                this->quadrants_.at(3).push_back(i);
            }
        }
    }

    // remove any empty quadrants
    const auto new_end(std::remove_if(this->quadrants_.begin(),
                    this->quadrants_.end(),
                    [] (const std::vector<uint32_t> & x_) {
                        return x_.size() == 0;
                    }));
    this->quadrants_.erase(new_end, this->quadrants_.end());

    this->num_quadrants_ = this->quadrants_.size();

    // check number of quadrants
    if (dim_x == 1 && dim_y == 1) {
        LOG(FATAL) << "Informants don't make sense with a 1 location network";
    } else if(dim_x == 1 || dim_y == 1) {
        CHECK(dim_x > 1 || dim_y > 1);
        CHECK_EQ(this->num_quadrants_, 2);
    } else {
        CHECK_GT(dim_x, 1);
        CHECK_GT(dim_y, 1);
        CHECK_EQ(this->num_quadrants_, 4);
    }
}


QuadrantInformant::QuadrantInformant(const QuadrantInformant & other)
    : Informant(other), num_quadrants_(other.num_quadrants_),
      quadrants_(other.quadrants_) {
}


const InformantTip QuadrantInformant::tip(const proto::Unit &evader_unit) {
    double reliable(0);
    double deceitful(0);
    double noisy(0);
	uint32_t r=random(0,3); 

    if (r == 1){
		reliable = 1;
	}
	if (r == 2 ){
		deceitful = 1;
	}
    if (r == 3){
		noisy = 1;
	}

    const double draw(this->rng()->runif_01());
    const uint32_t evader_loc(evader_unit.loc());
    std::vector<uint32_t> locs;
    if (draw < reliable) {
        // reliable informant

        // pick quadrant with evader
        for (uint32_t i = 0; i < this->num_quadrants_; ++i) {
            // if evader in quadrant, add quadrant to locs
            if (std::find(this->quadrants_.at(i).begin(),
                            this->quadrants_.at(i).end(),
                            evader_loc) != this->quadrants_.at(i).end()) {
                locs = this->quadrants_.at(i);
                break;
            }
        }
        CHECK_GT(locs.size(), 0);
    } else if(draw < (reliable + deceitful)) {
        // deceitful informant

        // pick uniformly over all quadrants without the evader
        std::vector<uint32_t> possible_quadrants;
        for (uint32_t i = 0; i < this->num_quadrants_; ++i) {
            // if evader not in quadrant, add quadrant to locs
            if (std::find(this->quadrants_.at(i).begin(),
                            this->quadrants_.at(i).end(),
                            evader_loc) == this->quadrants_.at(i).end()) {
                possible_quadrants.push_back(i);
            }
        }

        CHECK_EQ(possible_quadrants.size(), this->num_quadrants_ - 1);

        const uint32_t pick(
                possible_quadrants.at(
                        this->rng()->rint(0, possible_quadrants.size())));
        locs = this->quadrants_.at(pick);
    } else {
        // noisy informant

        // pick uniformly over all quadrants
        locs = this->quadrants_.at(this->rng()->rint(0, this->num_quadrants_));
    }

    return InformantTip(locs, reliable, deceitful, noisy);
}


void QuadrantInformant::reset() {
}





} // namespace coopPE
