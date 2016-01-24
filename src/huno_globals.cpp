#include "common_functions.h"

//Left/Right Leg/Arm logic
bool isRightArm(int joint_ID)
{
 if(joint_ID >= 13 && joint_ID <= 15)
 { return true; }
 else
 { return false; }
}

bool isLeftArm(int joint_ID)
{
 if(joint_ID >= 10 && joint_ID <= 12)
 { return true; }
 else
 { return false; }
}

bool isRightLeg(int joint_ID)
{
 if(joint_ID >= 5 && joint_ID <= 9)
 { return true; }
 else
 { return false; }
}

bool isLeftLeg(int joint_ID)
{
 if(joint_ID >= 0 && joint_ID <= 4)
 { return true; }
 else
 { return false; }
}

