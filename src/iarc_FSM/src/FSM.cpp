#include "FSM.h"
#include <cmath>

void FSM::calculateAngle(float x, float y)
{
	if(x == 0)
	{
		if(y >= 0)
			angle = 90;
		else
			angle = -90;
	}
	else
	{
		angle = atan(y/x)/PI*180;
	}

}

void FSM::changeAction(std::vector<float> &ObjPos)
{
    float x,y;
    if(ObjPos[0] == 0 && ObjPos[1] == 0)
    {
        act = NoDetectObject;
        angle = 0;
        return ;
    }
    x = halfWidth - ObjPos[0];
    y = halfHeight - ObjPos[1];
    calculateAngle(x, y);
    if(x == 0 && y == 0)
    {
    	act = OnObject;
    	return ;
    }
    else
    {
    	act = DetectObject;
    	return ;
    }
}

void FSM::changeStatus()
{
	switch(s)
	{
		case Up:
			if(act == DetectObject)
				s = FlyTo;
			else if(act == NoDetectObject)
				s = Up;
			else if(act == OnObject)
				s = Wait;
			break;
		case Down:
			if(act == DetectObject)
				s = FlyTo;
			else if(act == NoDetectObject)
				s = Wait;
			else if(act == OnObject)
				s = Down;
			break;
		case Wait:
			if(act == DetectObject)
				s = FlyTo;
			else if(act == NoDetectObject)
				s = Up;
			else if(act == OnObject)
				s = Down;
			break;
		case FlyTo:
			if(act == DetectObject)
				s = FlyTo;
			else if(act == NoDetectObject)
				s = Up;
			else if(act == OnObject)
				s = Down;
			break;
		default:
			break;
	}
}
