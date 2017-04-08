#include <iostream>
#include <vector>

#define halfWidth 320
#define halfHeight 240
#define PI 3.14159

enum Status
{
    Up,
    Down,
    Wait,
    FlyTo
};

enum Action
{
    DetectObject,
    NoDetectObject,
    OnObject,
    NoAction
};

class FSM
{
    //members
private:
    Status s;
    Action act;
    float angle;
    
    //funtions
    void calculateAngle(float x, float y);

public:
    FSM(){s = Wait; act = NoAction;};
    Status getStatus(){return s;};
    float getAngle(){return angle;};
    void changeStatus();
    void changeAction(std::vector<float> &ObjPos);  
};
