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
        (float) beta1,
        (float) beta2,
        (float) D,
	(float) Kp,
        (float) Kd,
	(float) M,
	(float) theta0,
        (float) K,
	(float) K1,
        (float) K2,
        (float) K3,
        (float) K4,
	(float) di,
	(float) li,
        (float) lambda_1,
        (float) lambda_2,
        (float) lambda_3,
        (float) e0,
        (float) alpha1,
        (float) alpha2,
        (float) etat1,
        (float) etat2,
        (float) offset,
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

/*-----------------------------------------previous paper------------------------------------------------------------------------*/
        Angle calcAcceleration(Angle dQ0, Angle dQ1, float delta); //equation 6
        Angle calcErrorPosition(Angle Q, Angle Qd);// E(t), primitive de equation 7
        Angle calcErrorVelocity(Angle dQ, Angle dQd);//equation 7
        Angle calcSlidingSurface(Angle dQWithError, Angle QWithError);//Equation 12, S(t)
        Angle sign(Angle sigma);// Equation 17
        Angle saturation(Angle sigma);
        Angle calcTorque(Angle ddQd, Angle dQWithError, Angle QWithError, Angle sigma);// equation 18
        void calcTDE(Angle& torque, Angle torqueT1, Angle ddQ1);// equation 5
        Angle calcAngle(Vector3a desiredAngle0, Vector3a desiredAngle1, Vector2a angle0, Vector2a angle1,  Vector2a angle2, Angle* testtorque);
        Angle calcD(Angle dQWithError, Angle QWithError);

/*------------------------------------------new paper-----------------------------------------------------------------------------*/
/*Fonctions utilisees*/
        void  verifMaxError(Angle& error);
        Angle calcS(Angle epsilonPoint, Angle epsilonLatino);//eq 14
        Angle calcEpsilonTilde(Angle fi, Angle epsilon);// peut ressembler a calcErrorPosition(), changer les valeurs entrees de la fonction. Eq 13 du nouveau paper
        Angle calcDEpsilonTilde(Angle epsilonTilde);//eq 15
        Angle calcGammaEqM(Angle ddQd, Angle H_hat);//eq 17
        Angle calcGammaFtM(Angle s); //eq 19 pas
        Angle calcGammaM(Angle gammaEqM, Angle gammaFtM);//eq 20
        Angle calcThetaCmd(Angle theta, Angle thetaPoint, Angle gammaM);//eq 21
        Angle sig(float exponent, Angle argument);

/*Fonction non-utilises*/
        Angle calcFi(Angle f0, Angle fInf, double l);//eq 11

/*Fonction prncipale*/
        Angle calcAngleNew(Vector3a desiredAngle0, Vector3a desiredAngle1, Vector2a angle0, Vector2a angle1,  Vector2a angle2, Angle* testtorque);
};
