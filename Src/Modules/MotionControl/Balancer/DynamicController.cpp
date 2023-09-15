//
// Created by vincent on 19-05-11.
//

#include "DynamicController.h"
#include "Tools/Math/Eigen.h"
#include "cstdio"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Math/Angle.h"

#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(DynamicController, motionControl);

void DynamicController::update(DynamicTesting& representation) {
    representation.init = [this, &representation]() -> Pose3f { return init(representation); };
    representation.addBalance = [this, &representation](JointAngles& jointRequest) -> bool
    {
        return addBalance(jointRequest, representation);
    };
  
    DECLARE_PLOT("module:DynamicController:lHipYawPitchBefore");
    DECLARE_PLOT("module:DynamicController:lHipYawPitchAfter");

    DECLARE_PLOT("module:DynamicController:lHipRollBefore");
    DECLARE_PLOT("module:DynamicController:lHipRollAfter");

    DECLARE_PLOT("module:DynamicController:lHipPitchBefore");
    DECLARE_PLOT("module:DynamicController:lHipPitchAfter");

    DECLARE_PLOT("module:DynamicController:lKneePitchBefore");
    DECLARE_PLOT("module:DynamicController:lKneePitchAfter");

    DECLARE_PLOT("module:DynamicController:lAnklePitchBefore");
    DECLARE_PLOT("module:DynamicController:lAnklePitchAfter");

    DECLARE_PLOT("module:DynamicController:lAnkleRollBefore");
    DECLARE_PLOT("module:DynamicController:lAnkleRollAfter");



    DECLARE_PLOT("module:DynamicController:rHipYawPitchBefore");
    DECLARE_PLOT("module:DynamicController:rHipYawPitchAfter");

    DECLARE_PLOT("module:DynamicController:rHipRollBefore");
    DECLARE_PLOT("module:DynamicController:rHipRollAfter");

    DECLARE_PLOT("module:DynamicController:rHipPitchBefore");
    DECLARE_PLOT("module:DynamicController:rHipPitchAfter");

    DECLARE_PLOT("module:DynamicController:rKneePitchBefore");
    DECLARE_PLOT("module:DynamicController:rKneePitchAfter");

    DECLARE_PLOT("module:DynamicController:rAnklePitchBefore");
    DECLARE_PLOT("module:DynamicController:rAnklePitchAfter");

    DECLARE_PLOT("module:DynamicController:rAnkleRollBefore");
    DECLARE_PLOT("module:DynamicController:rAnkleRollAfter");
    // OUTPUT_TEXT("Sliding Update");
}

DynamicController::DynamicController() {

}

Pose3f DynamicController::init(DynamicTesting& representation) {
    // OUTPUT_TEXT("Sliding Init");

    lastFrameTime = delta;
    delta = (theFrameInfo.time - lastFrameTime) * 0.001f;

    tampon2[0] = Vector2a(tampon1[0]);
    tampon2[1] = Vector2a(tampon1[1]);
    tampon2[2] = Vector2a(tampon1[2]);
    tampon2[3] = Vector2a(tampon1[3]);
    tampon2[4] = Vector2a(tampon1[4]);
    tampon2[5] = Vector2a(tampon1[5]);
    tampon2[6] = Vector2a(tampon1[6]);
    tampon2[7] = Vector2a(tampon1[7]);
    tampon2[8] = Vector2a(tampon1[8]);
    tampon2[9] = Vector2a(tampon1[9]);
    tampon2[10] = Vector2a(tampon1[10]);
    tampon2[11] = Vector2a(tampon1[11]);


    tampon1[0] = Vector2a(tampon[0]);
    tampon1[1] = Vector2a(tampon[1]);
    tampon1[2] = Vector2a(tampon[2]);
    tampon1[3] = Vector2a(tampon[3]);
    tampon1[4] = Vector2a(tampon[4]);
    tampon1[5] = Vector2a(tampon[5]);
    tampon1[6] = Vector2a(tampon[6]);
    tampon1[7] = Vector2a(tampon[7]);
    tampon1[8] = Vector2a(tampon[8]);
    tampon1[9] = Vector2a(tampon[9]);
    tampon1[10] = Vector2a(tampon[10]);
    tampon1[11] = Vector2a(tampon[11]);

    tampon[0] = Vector2a(theJointRequest.angles[Joints::lHipYawPitch],
                         (theJointRequest.angles[Joints::lHipYawPitch] - tampon1[0][0]) / delta);
    tampon[1] = Vector2a(theJointRequest.angles[Joints::lHipRoll],
                         (theJointRequest.angles[Joints::lHipRoll] - tampon1[1][0]) / delta);
    tampon[2] = Vector2a(theJointRequest.angles[Joints::lHipPitch],
                         (theJointRequest.angles[Joints::lHipPitch] - tampon1[2][0]) / delta);
    tampon[3] = Vector2a(theJointRequest.angles[Joints::lKneePitch],
                         (theJointRequest.angles[Joints::lKneePitch] - tampon1[3][0]) / delta);
    tampon[4] = Vector2a(theJointRequest.angles[Joints::lAnklePitch],
                         (theJointRequest.angles[Joints::lAnklePitch] - tampon1[4][0]) / delta);
    tampon[5] = Vector2a(theJointRequest.angles[Joints::lAnkleRoll],
                         (theJointRequest.angles[Joints::lAnkleRoll] - tampon1[5][0]) / delta);
    tampon[6] = Vector2a(theJointRequest.angles[Joints::rHipYawPitch],
                         (theJointRequest.angles[Joints::rHipYawPitch] - tampon1[6][0]) / delta);
    tampon[7] = Vector2a(theJointRequest.angles[Joints::rHipRoll],
                         (theJointRequest.angles[Joints::rHipRoll] - tampon1[7][0]) / delta);
    tampon[8] = Vector2a(theJointRequest.angles[Joints::rHipPitch],
                         (theJointRequest.angles[Joints::rHipPitch] - tampon1[8][0]) / delta);
    tampon[9] = Vector2a(theJointRequest.angles[Joints::rKneePitch],
                         (theJointRequest.angles[Joints::rKneePitch] - tampon1[9][0]) / delta);
    tampon[10] = Vector2a(theJointRequest.angles[Joints::rAnklePitch],
                          (theJointRequest.angles[Joints::rAnklePitch] - tampon1[10][0]) / delta);
    tampon[11] = Vector2a(theJointRequest.angles[Joints::rAnkleRoll],
                          (theJointRequest.angles[Joints::rAnkleRoll] - tampon1[11][0]) / delta);

    return Pose3f(0, 0, 0);
}

bool DynamicController::addBalance(JointAngles& desiredJointAngles, DynamicTesting& representation) {
    // OUTPUT_TEXT("Sliding add balance");
Angle torquetest;

    tamponDesired1[0] = Vector3a(tamponDesired[0]);
    tamponDesired1[1] = Vector3a(tamponDesired[1]);
    tamponDesired1[2] = Vector3a(tamponDesired[2]);
    tamponDesired1[3] = Vector3a(tamponDesired[3]);
    tamponDesired1[4] = Vector3a(tamponDesired[4]);
    tamponDesired1[5] = Vector3a(tamponDesired[5]);
    tamponDesired1[6] = Vector3a(tamponDesired[6]);
    tamponDesired1[7] = Vector3a(tamponDesired[7]);
    tamponDesired1[8] = Vector3a(tamponDesired[8]);
    tamponDesired1[9] = Vector3a(tamponDesired[9]);
    tamponDesired1[10] = Vector3a(tamponDesired[10]);
    tamponDesired1[11] = Vector3a(tamponDesired[11]);

    tamponDesired[0] = Vector3a(desiredJointAngles.angles[Joints::lHipYawPitch],
                                     (desiredJointAngles.angles[Joints::lHipYawPitch] - tamponDesired1[0][0]) / delta, 0);
    tamponDesired[1] = Vector3a(desiredJointAngles.angles[Joints::lHipRoll],
                                 (desiredJointAngles.angles[Joints::lHipRoll] - tamponDesired1[1][0]) / delta, 0);
    tamponDesired[2] = Vector3a(desiredJointAngles.angles[Joints::lHipPitch],
                                  (desiredJointAngles.angles[Joints::lHipPitch] - tamponDesired1[2][0]) / delta, 0);
    tamponDesired[3] = Vector3a(desiredJointAngles.angles[Joints::lKneePitch],
                                   (desiredJointAngles.angles[Joints::lKneePitch] - tamponDesired1[3][0]) / delta, 0);
    tamponDesired[4] = Vector3a(desiredJointAngles.angles[Joints::lAnklePitch],
                                    (desiredJointAngles.angles[Joints::lAnklePitch] - tamponDesired1[4][0]) / delta, 0);
    tamponDesired[5] = Vector3a(desiredJointAngles.angles[Joints::lAnkleRoll],
                                   (desiredJointAngles.angles[Joints::lAnkleRoll] - tamponDesired1[5][0]) / delta, 0);

    tamponDesired[6] = Vector3a(desiredJointAngles.angles[Joints::rHipYawPitch],
                                     (desiredJointAngles.angles[Joints::rHipYawPitch] - tamponDesired1[6][0]) / delta, 0);
    tamponDesired[7] = Vector3a(desiredJointAngles.angles[Joints::rHipRoll],
                                 (desiredJointAngles.angles[Joints::rHipRoll]  - tamponDesired1[7][0]) / delta, 0);
    tamponDesired[8] = Vector3a(desiredJointAngles.angles[Joints::rHipPitch],
                                  (desiredJointAngles.angles[Joints::rHipPitch] - tamponDesired1[8][0]) / delta, 0);
    tamponDesired[9] = Vector3a(desiredJointAngles.angles[Joints::rKneePitch],
                                   (desiredJointAngles.angles[Joints::rKneePitch] - tamponDesired1[9][0]) / delta, 0);
    tamponDesired[10] = Vector3a(desiredJointAngles.angles[Joints::rAnklePitch],
                                    (desiredJointAngles.angles[Joints::rAnklePitch] - tamponDesired1[10][0]) / delta, 0);
    tamponDesired[11] = Vector3a(desiredJointAngles.angles[Joints::rAnkleRoll],
                                   (desiredJointAngles.angles[Joints::rAnkleRoll] - tamponDesired1[11][0]) / delta, 0);

    PLOT("module:DynamicController:lHipYawPitchBefore",desiredJointAngles.angles[Joints::lHipYawPitch]);
    desiredJointAngles.angles[Joints::lHipYawPitch] = calcAngle(tamponDesired[0],tamponDesired1[0], tampon[0], tampon1[0], tampon2[0], &torquetest);
    PLOT("module:DynamicController:lHipYawPitchAfter", desiredJointAngles.angles[Joints::lHipYawPitch]);

    PLOT("module:DynamicController:lHipRollBefore",desiredJointAngles.angles[Joints::lHipRoll]);
    desiredJointAngles.angles[Joints::lHipRoll] = calcAngle(tamponDesired[1],tamponDesired1[1], tampon[1], tampon1[1], tampon2[1], &torquetest);
    PLOT("module:DynamicController:lHipRollAfter", desiredJointAngles.angles[Joints::lHipRoll]);
    PLOT("module:DynamicController:lHipPitchBefore",desiredJointAngles.angles[Joints::lHipPitch]);
    desiredJointAngles.angles[Joints::lHipPitch] = calcAngle(tamponDesired[2],tamponDesired1[2], tampon[2], tampon1[2], tampon2[2], &torquetest);
    PLOT("module:DynamicController:lHipPitchAfter", desiredJointAngles.angles[Joints::lHipPitch]);
    PLOT("module:DynamicController:lKneePitchBefore",desiredJointAngles.angles[Joints::lKneePitch]);
    desiredJointAngles.angles[Joints::lKneePitch] = calcAngle(tamponDesired[3],tamponDesired1[3], tampon[3], tampon1[3], tampon2[3], &torquetest);
    PLOT("module:DynamicController:lKneePitchAfter", desiredJointAngles.angles[Joints::lKneePitch]);
    PLOT("module:DynamicController:lAnklePitchBefore",desiredJointAngles.angles[Joints::lAnklePitch]);
    desiredJointAngles.angles[Joints::lAnklePitch] = calcAngle(tamponDesired[4],tamponDesired1[4], tampon[4], tampon1[4], tampon2[4], &torquetest);
    PLOT("module:DynamicController:lAnklePitchAfter", desiredJointAngles.angles[Joints::lAnklePitch]);
    PLOT("module:DynamicController:lAnkleRollBefore",desiredJointAngles.angles[Joints::lAnkleRoll]);
    desiredJointAngles.angles[Joints::lAnkleRoll] = calcAngle(tamponDesired[5],tamponDesired1[5], tampon[5], tampon1[5], tampon2[5], &torquetest);
    PLOT("module:DynamicController:lAnkleRollAfter", desiredJointAngles.angles[Joints::lAnkleRoll]);

    PLOT("module:DynamicController:rHipYawPitchBefore",desiredJointAngles.angles[Joints::rHipYawPitch]);
    desiredJointAngles.angles[Joints::rHipYawPitch] = calcAngle(tamponDesired[6],tamponDesired1[6], tampon[6], tampon1[6], tampon2[6], &torquetest);
    PLOT("module:DynamicController:rHipYawPitchAfter", desiredJointAngles.angles[Joints::rHipYawPitch]);
    PLOT("module:DynamicController:rHipRollBefore",desiredJointAngles.angles[Joints::rHipRoll]);
    desiredJointAngles.angles[Joints::rHipRoll] = calcAngle(tamponDesired[7],tamponDesired1[7], tampon[7], tampon1[7], tampon2[7], &torquetest);
    PLOT("module:DynamicController:rHipRollAfter", desiredJointAngles.angles[Joints::rHipRoll]);
    PLOT("module:DynamicController:rHipPitchBefore",desiredJointAngles.angles[Joints::rHipPitch]);
    desiredJointAngles.angles[Joints::rHipPitch] = calcAngle(tamponDesired[8],tamponDesired1[8], tampon[8], tampon1[8], tampon2[8], &torquetest);
    PLOT("module:DynamicController:rHipPitchAfter",desiredJointAngles.angles[Joints::rHipPitch]);
    PLOT("module:DynamicController:rKneePitchBefore", desiredJointAngles.angles[Joints::rKneePitch]);
    desiredJointAngles.angles[Joints::rKneePitch] = calcAngle(tamponDesired[9],tamponDesired1[9], tampon[9], tampon1[9], tampon2[9], &torquetest);
    PLOT("module:DynamicController:rKneePitchAfter",desiredJointAngles.angles[Joints::rKneePitch]);
    PLOT("module:DynamicController:rAnklePitchBefore", desiredJointAngles.angles[Joints::rAnklePitch]);
    desiredJointAngles.angles[Joints::rAnklePitch] = calcAngle(tamponDesired[10],tamponDesired1[10], tampon[10], tampon1[10], tampon2[10],&torquetest);
    PLOT("module:DynamicController:rAnklePitchAfter", desiredJointAngles.angles[Joints::rAnklePitch]);
    PLOT("module:DynamicController:rAnkleRollBefore",desiredJointAngles.angles[Joints::rAnkleRoll]);
    desiredJointAngles.angles[Joints::rAnkleRoll] = calcAngle(tamponDesired[11],tamponDesired1[11], tampon[11], tampon1[11], tampon2[11],&torquetest);
    PLOT("module:DynamicController:rAnkleRollAfter", desiredJointAngles.angles[Joints::rAnkleRoll]);

    // OUTPUT_TEXT("Angle desiré rAnkleRoll : " << desiredJointAngles.angles[Joints::rAnkleRoll]);
    return true;
}

Angle DynamicController::calcAcceleration(Angle dQ0, Angle dQ1, float delta){
    return (dQ0-dQ1)/delta;
}

Angle DynamicController::calcErrorPosition(Angle Q, Angle Qd){
    return Q - Qd;
}

Angle DynamicController::calcErrorVelocity(Angle dQ, Angle dQd){
    return dQ - dQd;
}

Angle DynamicController::calcSlidingSurface(Angle dQWithError, Angle QWithError){
    return dQWithError + (beta1+(1/(2*theta0))*D*D) * QWithError;

}


Angle DynamicController::sign(Angle sigma) {
    if(sigma > 0) {
        return 1;
    } else if (sigma < 0) {
        return -1;
    }
    return 0;
}

Angle DynamicController::saturation(Angle sigma){
    float phi = 0.8f;// chang/ de 0.8 a 1.2 pendant tests.
    float phi1 = 1/ phi;

    if(std::abs(sigma) > phi1){
        return sign(sigma);
    }
    return sigma*phi1;
}


Angle DynamicController::calcTorque(Angle ddQd, Angle dQWithError, Angle QWithError, Angle sigma){
       return A * (ddQd - QWithError - (beta1+(1/(2*theta0))*D*D) * dQWithError- beta2*(dQWithError + (beta1+(1/(2*theta0))*D*D)  * QWithError) - K * calcD(dQWithError, QWithError)*saturation(sigma));
}

void DynamicController::calcTDE(Angle& torque, Angle torqueT1, Angle ddQ1){
    Angle torque1 = torqueT1 - A * ddQ1; // torque1 = torque(t-1) -A * ddQ(t-1)
    torque = torque + torque1;
}

Angle DynamicController::calcTorqueToPosition(Angle Q, Angle dQ, Angle dQd, Angle torque){
    return Q + (1/Kp) * (Kd*(dQ-dQd) - torque);
}

Angle DynamicController::calcAngle(Vector3a desiredAngle0, Vector3a desiredAngle1, Vector2a angle0, Vector2a angle1,  Vector2a angle2, Angle* testtorque) {
    Angle Qd = desiredAngle0[0];
    Angle dQd = desiredAngle0[1];
    Angle Q = angle0[0];
    Angle dQ = angle0[1];
    //Angle ddQ = calcAcceleration(dQ,angle1[1], delta);
    Angle ddQ1 = calcAcceleration(angle1[1],angle2[1], delta);

    Angle QWithError = calcErrorPosition(Q, Qd);
    Angle dQWithError = calcErrorVelocity(dQ, dQd);

    Angle sigma = calcSlidingSurface(dQWithError, QWithError);

    Angle ddQd = calcAcceleration(dQd,desiredAngle1[1], delta); // dQd(t) - dQd(t-1) / delta

    desiredAngle0[2] = calcTorque(ddQd,dQWithError, QWithError, sigma);
    *testtorque=desiredAngle0[2];
    calcTDE(desiredAngle0[2],desiredAngle1[2], ddQ1); // torque(t) = desiredAngle0[2] , torque(t-1) = desiredAngle1[2]
    Angle test = Angle(calcTorqueToPosition(Q, dQ,dQd, desiredAngle0[2]));
    // OUTPUT_TEXT("Angle desiré avant : " << desiredAngle0[0] << " Angle desiré apres : " << test);
   // PLOT("module:DynamicController:calctorque",desiredAngle0[2]);
    return test;
}

Angle DynamicController::calcD(Angle dQWithError, Angle QWithError) {

	return 1 / (float)(di+(1-di)*(exp(-pow(fabs(dQWithError + (beta1+(1/(2*theta0))*D*D) * QWithError),li))));
}