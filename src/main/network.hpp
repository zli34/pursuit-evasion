#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <memory>
#include <cstdint>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include "coopPE.pb.h"

namespace coopPE {

class Network {
private:
    proto::NetworkInit init_;

    // number of nodes
    uint32_t num_nodes_;
    // list of nodes
    proto::NodeList node_list_;

    // adjacency matrix
    // row to column
    boost::numeric::ublas::mapped_matrix<uint32_t> adj_;

    // distance between nodes
    std::vector<std::vector<uint32_t> > dist_;

    // neighbors
    std::vector<std::vector<uint32_t> > neigh_;

    // calculate distance for entire network
    std::vector<std::vector<uint32_t> > calc_dist() const;

    // generate a grid type network
    static std::shared_ptr<Network> gen_grid(
            const uint32_t dim_x, const uint32_t dim_y, const bool wrap);
public:
    std::shared_ptr<Network> clone() const;

    // number of nodes
    uint32_t size() const;

    // Retrieve index-th node in the network
    const proto::Node & node(const uint32_t index) const;

    // Retrieve the adjacency matrix
    const boost::numeric::ublas::mapped_matrix<uint32_t> & adj() const;

    const std::vector<std::vector<uint32_t> > & dist() const;

    const std::vector<uint32_t> & neigh(const uint32_t & i) const;

    uint32_t dist(const uint32_t & a, const uint32_t & b) const;

    // generate a network from NetworkInit data
    static std::shared_ptr<Network> gen_network(
            const proto::NetworkInit & init);

    const proto::NetworkInit & init() const;

    const proto::NodeList & node_list() const;
};

} // namespace coopPE


#endif // NETWORK_HPP
