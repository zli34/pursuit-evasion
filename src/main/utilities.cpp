#include "utilities.hpp"

#include <queue>

#include <glog/logging.h>

namespace coopPE {


bool next_move(std::vector<proto::Unit_Action> & moves) {
    const uint32_t num_units(moves.size());
    // increment action and return, if can't increment (already at
    // RIGHT) then reset (set to NOTHING) and move to next pursuer
    // (don't return immediately)
    for (uint32_t i = 0; i < num_units; ++i) {
        if (moves.at(i) == proto::Unit_Action_NOTHING) {
            moves.at(i) = proto::Unit_Action_UP;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_UP) {
            moves.at(i) = proto::Unit_Action_DOWN;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_DOWN) {
            moves.at(i) = proto::Unit_Action_LEFT;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_LEFT) {
            moves.at(i) = proto::Unit_Action_RIGHT;
            return true;
        } else if (moves.at(i) == proto::Unit_Action_RIGHT) {
            moves.at(i) = proto::Unit_Action_NOTHING;
        } else {
            LOG(FATAL) << "Unknown value for action: " << moves.at(i);
        }
    }
    // all pursuers were already set to RIGHT, this is the last set of
    // actions, thus return false to indicate no more moves,
    return false;
}


std::vector<uint32_t> find_peaks(
        const std::vector<double> & probs,
        const uint32_t & num_peaks,
        const std::shared_ptr<const Network> & network) {
    std::vector<uint32_t> peaks;


    std::vector<std::pair<double, uint32_t> > coverages;
    for (uint32_t i = 0; i < network->size(); ++i) {
        double c(probs.at(i));

        std::for_each(network->neigh(i).begin(),
                network->neigh(i).end(),
                [&c, &probs] (const uint32_t & n_) {
                    c += probs.at(n_);
                });

        coverages.emplace_back(std::move(c), i);
    }

    // sort coverage pairs
    for (uint32_t i = 0; i < num_peaks; ++i) {
        auto pk(std::max_element(coverages.begin(), coverages.end()));

        if (pk->first > 1e-6) {
            peaks.push_back(pk->second);
            pk->first = 0.0;

            if (i < (num_peaks - 1)) {
                auto neigh_it(network->neigh(pk->second).begin());
                const auto neigh_end(network->neigh(pk->second).end());
                for (; neigh_it != neigh_end; ++neigh_it) {
                    // zero out neighbor coverage
                    coverages.at(*neigh_it).first = 0.0;
                    auto second_neigh_it(
                            network->neigh(*neigh_it).begin());
                    const auto second_neigh_end(
                            network->neigh(*neigh_it).end());
                    for (; second_neigh_it != second_neigh_end;
                         ++second_neigh_it) {
                        // subtract neighbor probability from second neighbors
                        coverages.at(*second_neigh_it).first -=
                            probs.at(*neigh_it);
                    }
                }
            }
        } else {
            CHECK_GT(i, 0);
            peaks.push_back(peaks.at(0));
        }
    }

    return peaks;
}



std::vector<uint32_t> assign_peaks(
        const std::vector<uint32_t> & locs,
        const std::vector<uint32_t> & peaks,
        const std::shared_ptr<const Network> & network) {
    const uint32_t num_peaks(peaks.size());
    CHECK_EQ(locs.size(), num_peaks);

    std::vector<uint32_t> assignments;
    for (uint32_t i = 0; i < num_peaks; ++i) {
        assignments.push_back(i);
    }

    std::vector<uint32_t> best_assignments;
    double best_dist = std::numeric_limits<double>::infinity();
    do {
        double dist = 0.0;
        for (uint32_t i = 0; i < num_peaks; ++i) {
            dist += network->dist(peaks.at(assignments.at(i)),
                    locs.at(i));
        }

        if (dist < best_dist) {
            best_assignments = assignments;
            best_dist = dist;
        }
    } while (std::next_permutation(assignments.begin(), assignments.end()));

    return best_assignments;
}




} // namespace coopPE
