#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#define DEG2RAD (3.14/180)

#define HUNO_NUM_JOINTS 16

/* Huno limb enumeration */
typedef enum
{
 LeftLeg,
 RightLeg,
 LeftArm,
 RightArm
} hunolimb_t;

// TODO: Change isLimb logic to a single IdLimb(int jointID) function
//       to return enum above
//hunolimb_t IdentifyLimb(int jointId);

/* Left/Right Leg/Arm logic */
bool isRightArm(int joint_ID);
bool isLeftArm(int joint_ID);
bool isRightLeg(int joint_ID);
bool isLeftLeg(int joint_ID);

#endif
