#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#define DEG2RAD (3.14/180)

//Left/Right Leg/Arm logic
bool isRightArm(int joint_ID);
bool isLeftArm(int joint_ID);
bool isRightLeg(int joint_ID);
bool isLeftLeg(int joint_ID);

#endif
