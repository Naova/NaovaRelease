//
// Created by vincent on 19-05-11.
//

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/DynamicTesting.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/FootGroundContactState.h"
#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Module/Module.h"
#include "Tools/Range.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Angle.h"

MODULE(DynamicController,
{,
    REQUIRES(FootSupport),
    REQUIRES(InertialSensorData),
    REQUIRES(FrameInfo),
    REQUIRES(RobotModel),
    REQUIRES(InertialData),
    REQUIRES(RobotDimensions),
    REQUIRES(KeyStates),
    REQUIRES(JointLimits),
    REQUIRES(MassCalibration),
    REQUIRES(JointAngles),
    REQUIRES(DamageConfigurationBody),
    USES(JointRequest),
    PROVIDES(DynamicTesting),
    LOADS_PARAMETERS(
    {,
     (float)beta1,
        (float)beta2,
        (float)D,
	(float) Kp,
        (float) Kd,
	 (float) A,
	(float) theta0,
	  (float) K,
	(float) di,
	(float) li,
    }),
});


class DynamicController : public DynamicControllerBase
{
public:
    void update(DynamicTesting& representation);
    DynamicController();

    /**
     * Implementation of Dynamic::init
     */
    Pose3f init(DynamicTesting& representation);

    /**
     * Implementation of Dynamic::addBalance
     */
    bool addBalance(JointAngles& jointAngles, DynamicTesting& representation);

private:
    float delta = 0;
    float lastFrameTime = 0;
    // t-2
    Vector2a tampon[12] = {
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero()
    };
    // t-1
    Vector2a tampon1[12] = {
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero()
    };
    // t-2
    Vector2a tampon2[12] = {
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero(),
            Vector2a::Zero()
    };
    Vector3a tamponDesired[12] = {
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero()
    };
    // t-1
    Vector3a tamponDesired1[12] = {
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero(),
            Vector3a::Zero()
    };
    Angle calcAcceleration(Angle dQ0, Angle dQ1, float delta);
    Angle calcErrorPosition(Angle Q, Angle Qd);
    Angle calcErrorVelocity(Angle dQ, Angle dQd);
    Angle calcSlidingSurface(Angle dQWithError, Angle QWithError);
    Angle calcTorque(Angle ddQd, Angle dQWithError, Angle QWithError, Angle sigma);
    void calcTDE(Angle& torque, Angle torqueT1, Angle ddQ1);
    Angle calcTorqueToPosition(Angle Q, Angle dQ, Angle dQd, Angle torque);
    Angle calcAngle(Vector3a desiredAngle0,Vector3a desiredAngle1, Vector2a angle0, Vector2a angle1,  Vector2a angle2, Angle* testtorque);
    Angle saturation(Angle sigma);
    Angle sign(Angle sigma);
	Angle calcD(Angle dQWithError, Angle QWithError);
};
