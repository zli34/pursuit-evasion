#include "network.hpp"
#include <glog/logging.h>

#include <set>

namespace coopPE {

std::shared_ptr<Network> Network::clone() const {
    return std::shared_ptr<Network>(new Network(*this));
}

uint32_t Network::size() const {
    return this->num_nodes_;
}

const proto::Node & Network::node(const uint32_t index) const {
    return this->node_list_.nodes(index);
}

std::vector<std::vector<uint32_t> > Network::calc_dist() const{
    CHECK_EQ(this->adj_.size1(), this->num_nodes_);
    CHECK_EQ(this->adj_.size2(), this->num_nodes_);

    std::vector<std::vector<uint32_t> > dist(this->num_nodes_,
            // divide max by 2 to prevent overflow
            std::vector<uint32_t>(this->num_nodes_,
                    std::numeric_limits<uint32_t>::max() / 2));

    for (uint32_t i = 0; i < this->num_nodes_; ++i) {
        dist.at(i).at(i) = 0.0;
        for (uint32_t j = 0; j < this->num_nodes_; ++j) {
            if (this->adj_(i,j) == 1 && i != j) {
                dist.at(i).at(j) = 1.0;
            }
        }
    }

    for (uint32_t i = 0; i < this->num_nodes_; ++i) {
        for (uint32_t j = 0; j < this->num_nodes_; ++j) {
            uint32_t & direct(dist.at(i).at(j));
            for (uint32_t k = 0; k < this->num_nodes_; ++k) {
                const uint32_t & step_one(dist.at(i).at(k));
                const uint32_t & step_two(dist.at(k).at(j));

                if (direct > (step_one + step_two)) {
                    direct = step_one + step_two;
                }
            }
        }
    }

    return dist;
}

const std::vector<std::vector<uint32_t> > & Network::dist() const {
    return this->dist_;
}


uint32_t Network::dist(const uint32_t & a, const uint32_t & b) const {
    return this->dist_.at(a).at(b);
}


const std::vector<uint32_t> & Network::neigh(const uint32_t & i) const {
    return this->neigh_.at(i);
}


std::shared_ptr<Network> Network::gen_network(
        const proto::NetworkInit & init) {
    CHECK(init.has_type());

    std::shared_ptr<Network> network(NULL);

    // call appropriate initializer
    switch (init.type()) {
    case proto::NetworkInit_NetType_GRID: {
        CHECK(init.has_dim_x());
        CHECK(init.has_dim_y());
        CHECK(init.has_wrap());

        network = gen_grid(init.dim_x(),init.dim_y(),init.wrap());
        break;
    }
    default:
        LOG(FATAL) << "Don't know how to initialize network of type "
                   << init.type() << ".";
        break;
    }

    network->init_ = init;

    // fill distance matrix
    network->dist_ = network->calc_dist();

    // get neighbors
    network->neigh_.resize(network->size(), std::vector<uint32_t>());
    for (uint32_t i = 0; i < network->size(); ++i) {
        const proto::Node & node(network->node(i));
        if (node.left() != i)
            network->neigh_.at(i).push_back(node.left());
        if (node.right() != i)
            network->neigh_.at(i).push_back(node.right());
        if (node.up() != i)
            network->neigh_.at(i).push_back(node.up());
        if (node.down() != i)
            network->neigh_.at(i).push_back(node.down());

        // remove duplicates
        std::sort(network->neigh_.at(i).begin(), network->neigh_.at(i).end());
        network->neigh_.at(i).erase(std::unique(
                        network->neigh_.at(i).begin(),
                        network->neigh_.at(i).end()),
                network->neigh_.at(i).end());
    }

    return network;
}


std::shared_ptr<Network> Network::gen_grid(
        const uint32_t dim_x, const uint32_t dim_y, const bool wrap) {
    std::shared_ptr<Network> network = std::make_shared<Network>();

    // Either dimension of 1 is not a grid.  Define this in another
    // network.  Such as a "Line Netowrk".
    CHECK_GT(dim_x,1);
    CHECK_GT(dim_y,1);

    // iterate through grid column first
    network->num_nodes_ = dim_x * dim_y;
    network->adj_ = boost::numeric::ublas::mapped_matrix<uint32_t>(
            network->num_nodes_,network->num_nodes_);

    for (uint32_t x = 0, i = 0; x < dim_x; ++x) {
        for (uint32_t y = 0; y < dim_y; ++y, ++i) {
            proto::Node * n = network->node_list_.add_nodes();
            n->set_index(i);
            n->set_x(static_cast<double>(x)/(dim_x - 1));
            n->set_y(static_cast<double>(y)/(dim_y - 1));

            // nothing action
            network->adj_(i,i) = 1;

            // up
            if (y > 0) {
                const uint32_t neigh = i-1;
                n->set_up(neigh);
                network->adj_(i,neigh) = 1;
            } else if(wrap) {
                const uint32_t neigh = i + dim_y - 1;
                n->set_up(neigh);
                network->adj_(i,neigh) = 1;
            } else {
                n->set_up(i);
            }

            // down
            if (y < (dim_y - 1)) {
                const uint32_t neigh = i + 1;
                n->set_down(neigh);
                network->adj_(i,neigh) = 1;
            } else if (wrap) {
                const uint32_t neigh = i - dim_y + 1;
                n->set_down(neigh);
                network->adj_(i,neigh) = 1;
            } else {
                n->set_down(i);
            }

            // left
            if (x > 0) {
                const uint32_t neigh = i - dim_y;
                n->set_left(neigh);
                network->adj_(i,neigh) = 1;
            } else if (wrap) {
                const uint32_t neigh = network->num_nodes_ - dim_y + y;
                n->set_left(neigh);
                network->adj_(i,neigh) = 1;
            } else {
                n->set_left(i);
            }

            // right
            if (x < (dim_x - 1)) {
                const uint32_t neigh = i + dim_y;
                n->set_right(neigh);
                network->adj_(i,neigh) = 1;
            } else if (wrap) {
                const uint32_t neigh = y;
                n->set_right(neigh);
                network->adj_(i,neigh) = 1;
            } else {
                n->set_right(i);
            }
        }

    }

    return network;
}


const proto::NetworkInit & Network::init() const {
    return this->init_;
}


const proto::NodeList & Network::node_list() const {
    return this->node_list_;
}



} // namespace coopPE
