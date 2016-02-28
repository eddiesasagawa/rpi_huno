#include "huno_globals.h"

hunolimb_t Identifylimb(int jointId)
{
  hunolimb_t limbRes;

  if      ( isLeftLeg(jointId) )
  { limbRes = LeftLeg; }
  else if ( isRightLeg(jointId) )
  { limbRes = RightLeg; }
  else if ( isLeftArm(jointId) )
  { limbRes = LeftArm; }
  else if ( isRightArm(jointId) )
  { limbRes = RightArm; }
  else // unknown id
  { limbRes = UnknownLimb; }

  return limbRes;
}
