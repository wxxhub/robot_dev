#ifndef _BEHAVIOR_MODEL_H_
#define _BEHAVIOR_MODEL_H_

namespace robot_behavior
{

class BehaviorModel
{
public:
    BehaviorModel(){}
    virtual ~BehaviorModel(){}

    virtual void setEnable(){}
    virtual void setDisable(){}

protected:
    bool enable_;
};

}


#endif /* _BEHAVIOR_MODEL_H_ */