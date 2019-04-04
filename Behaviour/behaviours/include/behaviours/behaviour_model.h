#ifndef _BEHAVIOUR_MODEL_H_
#define _BEHAVIOUR_MODEL_H_

namespace robot_behaviour
{

class BehaviourModel
{
public:
    BehaviourModel();
    virtual ~BehaviourModel();
    virtual void setDemoEnable(){}
    virtual void setDemoDisable(){}

protected:
    bool enable_;
};

}


#endif /* _BEHAVIOUR_MODEL_H_ */