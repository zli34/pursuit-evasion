#include "runner.hpp"

namespace coopPE {


proto::Outcome runner(System * s, Pursuer * p, Evader * e, InformantTimer * i,
        const uint32_t & time_points) {
    // reset
    s->reset();
    p->reset();
    e->reset();
    i->reset();

    // set starting locations
    s->pursuer_set_locs(p->starting_locs());
    s->evader_set_loc(e->starting_loc());

    uint32_t t = 0;
    while (!e->at_goal(s->evader_unit()) && !s->caught() && t < time_points) {
        // informant tip
        const InformantTip tip(i->generate_tip(s->evader_unit()));
        if (tip.has_tip()) {
            s->informant_set_tip(tip);
        }

        // get pursuer moves
        s->pursuer_move(p->decide_moves(
                        s->pursuer_units(), s->pursuer_history(),
                        s->tip_history()));

        // get evader moves
        s->evader_move(e->decide_move(
                        s->evader_unit(), s->evader_history()));

        // move
        s->move_all_units();
        ++t;
    }

    // return outcome value
    if (e->at_goal(s->evader_unit())) {
        return proto::Outcome::GOAL;
    } else if(s->caught()) {
        return proto::Outcome::CAUGHT;
    } else {
        return proto::Outcome::TIME;
    }
}


} // namespace coopPE
