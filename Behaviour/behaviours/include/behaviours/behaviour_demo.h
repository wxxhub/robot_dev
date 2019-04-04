#ifndef _BEHAVIOUR_DEMO_H_
#define _BEHAVIOUR_DEMO_H_

#include "behaviour_model.h"

namespace robot_behaviour
{

class BehaviourDemo : public robot_behaviour::BehaviourModel
{
public:
    BehaviourDemo();
    ~BehaviourDemo();
};

}

#endif  /* _BEHAVIOUR_DEMO_H_ */