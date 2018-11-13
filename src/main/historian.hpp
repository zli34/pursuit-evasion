#ifndef HISTORIAN_HPP
#define HISTORIAN_HPP

#include <glog/logging.h>
#include <mutex>

#include "network.hpp"
#include "informant.hpp"
#include "coopPE.pb.h"

namespace coopPE {


class Historian {
protected:
    const std::shared_ptr<const Network> network_;

    proto::SimStudyData ssd_;

    uint32_t num_reps_;

    std::mutex mtx_;

public:
    Historian(const std::shared_ptr<const Network> & network);

    Historian(const Historian & other) = delete;

    virtual ~Historian() = default;

    uint32_t add_rep(const std::vector<proto::Unit> & evader_history,
            const std::vector<std::vector<proto::Unit> > & pursuer_history,
            const std::vector<std::vector<double> > & evader_post_history,
            const std::map<uint32_t, InformantTip> & tip_history,
            const proto::Outcome & outcome, const uint32_t & num_points);

    template <typename T>
    void write_data(T & t) {
        std::lock_guard<std::mutex> lock(this->mtx_);
        std::string str;
        this->ssd_.SerializeToString(&str);
        t << str;
    }
};


} // namespace coopPE


#endif // HISTORIAN_HPP
