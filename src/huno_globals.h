#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#define DEG2RAD (3.14/180)
#define RAD2DEG (180/3.14)

#define HUNO_NUM_JOINTS 16

/* Huno limb enumeration, assigned to first joint ID of each limb */
typedef enum
{
 LeftLeg      = 0,
 RightLeg     = 5,
 LeftArm      = 10,
 RightArm     = 13,
 UnknownLimb  = -1
} hunolimb_t;

/* Return limb enum that input joint belongs to */
hunolimb_t IdentifyLimb(int jointId);

/* Left/Right Leg/Arm logic */
inline bool isLeftLeg(int jointId)
{
  return (jointId >= 0 && jointId <= 4);
}

inline bool isRightLeg(int jointId)
{
  return (jointId >= 5 && jointId <= 9);
}

inline bool isLeftArm(int jointId)
{
  return (jointId >= 10 && jointId <= 12);
}

inline bool isRightArm(int jointId)
{ 
  return (jointId >= 13 && jointId <= 15); 
}

#endif
