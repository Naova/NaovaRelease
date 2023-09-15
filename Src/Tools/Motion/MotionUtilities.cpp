/**
 * @file MotionUtilities.cpp
 * provides functions for interacting with motions
 * @author Bernd Poppinga
 */

#include "MotionUtilities.h"

void  MotionUtilities::copy(const JointRequest& source, JointRequest& target,
                            const StiffnessSettings& theStiffnessSettings,
                            const Joints::Joint startJoint, const Joints::Joint endJoint)
{
  for(int i = startJoint; i <= endJoint; ++i)
  {
    if(source.angles[i] != static_cast<float>(JointAngles::ignore))
      target.angles[i] = source.angles[i];
    target.stiffnessData.stiffnesses[i] = target.angles[i] != static_cast<float>(JointAngles::off) ? source.stiffnessData.stiffnesses[i] : 0;
    if(target.stiffnessData.stiffnesses[i] == StiffnessData::useDefault)
      target.stiffnessData.stiffnesses[i] = theStiffnessSettings.stiffnesses[i];
  }
}

void  MotionUtilities::interpolate(const JointRequest& from, const JointRequest& to,
                                   float fromRatio, JointRequest& target, bool interpolateStiffness,
                                   const StiffnessSettings& theStiffnessSettings, const JointAngles& lastJointAngles,
                                   const Joints::Joint startJoint, const Joints::Joint endJoint)
{
  for(int i = startJoint; i <= endJoint; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if(t == static_cast<float>(JointAngles::ignore) && f == static_cast<float>(JointAngles::ignore))
      continue;

    if(t == static_cast<float>(JointAngles::ignore))
      t = target.angles[i];
    if(f == static_cast<float>(JointAngles::ignore))
      f = target.angles[i];

    int fStiffness = f != static_cast<float>(JointAngles::off) ? from.stiffnessData.stiffnesses[i] : 0;
    int tStiffness = t != static_cast<float>(JointAngles::off) ? to.stiffnessData.stiffnesses[i] : 0;
    if(fStiffness == StiffnessData::useDefault)
      fStiffness = theStiffnessSettings.stiffnesses[i];
    if(tStiffness == StiffnessData::useDefault)
      tStiffness = theStiffnessSettings.stiffnesses[i];

    if(t == static_cast<float>(JointAngles::off) || t == static_cast<float>(JointAngles::ignore))
      t = lastJointAngles.angles[i];
    if(f == static_cast<float>(JointAngles::off) || f == static_cast<float>(JointAngles::ignore))
      f = lastJointAngles.angles[i];
    if(target.angles[i] == static_cast<float>(JointAngles::off) || target.angles[i] == static_cast<float>(JointAngles::ignore))
      target.angles[i] = lastJointAngles.angles[i];

    ASSERT(target.angles[i] != static_cast<float>(JointAngles::off) && target.angles[i] != static_cast<float>(JointAngles::ignore));
    ASSERT(t != static_cast<float>(JointAngles::off) && t != static_cast<float>(JointAngles::ignore));
    ASSERT(f != static_cast<float>(JointAngles::off) && f != static_cast<float>(JointAngles::ignore));

    target.angles[i] += -fromRatio * t + fromRatio * f;
    if(interpolateStiffness)
      target.stiffnessData.stiffnesses[i] += int(-fromRatio * float(tStiffness) + fromRatio * float(fStiffness));
    else
      target.stiffnessData.stiffnesses[i] = tStiffness;
  }
}

bool MotionUtilities::interpolate(JointRequest& joints, const float alpha, const float threshold, const JointRequest& theJointRequest, const JointAngles& theJointAngles,
                                  const JointRequest& theStandLegRequest)
{
  JointAngles lastAngles = theJointRequest.isValid() ? theJointRequest : theJointAngles;
  float diff = 0;

  for(int j = 0; j < Joints::numOfJoints; j++)
  {
    if(lastAngles.angles[j] == static_cast<float>(JointAngles::off) || lastAngles.angles[j] == static_cast<float>(JointAngles::ignore))
      lastAngles.angles[j] = theJointAngles.angles[j];

    if(theStandLegRequest.angles[j] == static_cast<float>(JointAngles::off) || theStandLegRequest.angles[j] == static_cast<float>(JointAngles::ignore))
      joints.angles[j] = lastAngles.angles[j];
    else
    {
      joints.angles[j] = lastAngles.angles[j] * (1.f - alpha) + theStandLegRequest.angles[j] * alpha;
      diff += std::abs(joints.angles[j] - theStandLegRequest.angles[j]);
    }
    joints.stiffnessData.stiffnesses[j] = 100;
  }
  diff /= static_cast<float>(Joints::numOfJoints);
  return diff < threshold;
}

void MotionUtilities::interpolate(const JointAngles& from, const JointRequest& to, float& ratio, JointRequest& target, const JointAngles& theJointAngles)
{
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if(t == static_cast<float>(JointAngles::ignore) && f == static_cast<float>(JointAngles::ignore))
      continue;

    if(t == static_cast<float>(JointAngles::ignore))
      t = target.angles[i];
    if(f == static_cast<float>(JointAngles::ignore))
      f = target.angles[i];

    if(t == static_cast<float>(JointAngles::off) || t == static_cast<float>(JointAngles::ignore))
      t = theJointAngles.angles[i];
    if(f == static_cast<float>(JointAngles::off) || f == static_cast<float>(JointAngles::ignore))
      f = theJointAngles.angles[i];

    target.angles[i] = ratio * (t - f) + f;

    target.stiffnessData.stiffnesses[i] = to.stiffnessData.stiffnesses[i];
  }
}

void MotionUtilities::stand(JointRequest& output)
{
  output.angles[Joints::lKneePitch] = 48.4503_deg;
  output.angles[Joints::rKneePitch] = 48.4503_deg;
  output.angles[Joints::lHipPitch] = -21.7585_deg;
  output.angles[Joints::rHipPitch] = -21.7585_deg;
  output.angles[Joints::lHipRoll] = 0_deg;
  output.angles[Joints::rHipRoll] = 0_deg;
  output.angles[Joints::lAnkleRoll] = 0_deg;
  output.angles[Joints::rAnkleRoll] = 0_deg;
  output.angles[Joints::rAnklePitch] = -26.6918_deg;
  output.angles[Joints::lAnklePitch] = -26.6918_deg;

  output.angles[Joints::rShoulderPitch] = 90_deg;
  output.angles[Joints::lShoulderPitch] = 90_deg;
  output.angles[Joints::rShoulderRoll] = -7_deg;
  output.angles[Joints::lShoulderRoll] = 7_deg;
  output.angles[Joints::rElbowYaw] = 0_deg;
  output.angles[Joints::lElbowYaw] = 0_deg;
  output.angles[Joints::lElbowRoll] = 0_deg;
  output.angles[Joints::rElbowRoll] = 0_deg;
  output.angles[Joints::rWristYaw] = 90_deg;
  output.angles[Joints::lWristYaw] = -90_deg;
}


void MotionUtilities::sit(JointRequest& output)
{
  //From B-Human 2019
  // Sit down to reduce the impact-force
  output.angles[Joints::lKneePitch] = 123_deg;
  output.angles[Joints::rKneePitch] = 123_deg;
  output.angles[Joints::lHipPitch] = -90_deg;
  output.angles[Joints::rHipPitch] = -90_deg;
  output.angles[Joints::lHipRoll] = 0_deg;
  output.angles[Joints::rHipRoll] = 0_deg;
  output.angles[Joints::lAnklePitch] = -45_deg;
  output.angles[Joints::rAnklePitch] = -45_deg;
  output.angles[Joints::lAnkleRoll] = 0_deg;
  output.angles[Joints::rAnkleRoll] = 0_deg;
}

