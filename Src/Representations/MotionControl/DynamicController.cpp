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
    delta = (theFrameInfo.time - lastFrameTime) * 0.001f;//time difference between the current frame and the last frame in seconds

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
    desiredJointAngles.angles[Joints::lHipYawPitch] = calcAngleNew(tamponDesired[0],tamponDesired1[0], tampon[0], tampon1[0], tampon2[0], &torquetest);
    PLOT("module:DynamicController:lHipYawPitchAfter", (desiredJointAngles.angles[Joints::lHipYawPitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:lHipRollBefore",desiredJointAngles.angles[Joints::lHipRoll]);
    desiredJointAngles.angles[Joints::lHipRoll] = calcAngleNew(tamponDesired[1],tamponDesired1[1], tampon[1], tampon1[1], tampon2[1], &torquetest);
    PLOT("module:DynamicController:lHipRollAfter", (desiredJointAngles.angles[Joints::lHipRoll])* (180.0 / M_PI));

    PLOT("module:DynamicController:lHipPitchBefore",desiredJointAngles.angles[Joints::lHipPitch]);
    desiredJointAngles.angles[Joints::lHipPitch] = calcAngleNew(tamponDesired[2],tamponDesired1[2], tampon[2], tampon1[2], tampon2[2], &torquetest);
    PLOT("module:DynamicController:lHipPitchAfter", (desiredJointAngles.angles[Joints::lHipPitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:lKneePitchBefore",desiredJointAngles.angles[Joints::lKneePitch]);
    desiredJointAngles.angles[Joints::lKneePitch] = calcAngleNew(tamponDesired[3],tamponDesired1[3], tampon[3], tampon1[3], tampon2[3], &torquetest);
    PLOT("module:DynamicController:lKneePitchAfter", (desiredJointAngles.angles[Joints::lKneePitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:lAnklePitchBefore",desiredJointAngles.angles[Joints::lAnklePitch]);
    desiredJointAngles.angles[Joints::lAnklePitch] = calcAngleNew(tamponDesired[4],tamponDesired1[4], tampon[4], tampon1[4], tampon2[4], &torquetest);
    PLOT("module:DynamicController:lAnklePitchAfter", (desiredJointAngles.angles[Joints::lAnklePitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:lAnkleRollBefore",desiredJointAngles.angles[Joints::lAnkleRoll]);
    desiredJointAngles.angles[Joints::lAnkleRoll] = calcAngleNew(tamponDesired[5],tamponDesired1[5], tampon[5], tampon1[5], tampon2[5], &torquetest);
    PLOT("module:DynamicController:lAnkleRollAfter", (desiredJointAngles.angles[Joints::lAnkleRoll])* (180.0 / M_PI));

    PLOT("module:DynamicController:rHipYawPitchBefore",desiredJointAngles.angles[Joints::rHipYawPitch]);
    desiredJointAngles.angles[Joints::rHipYawPitch] = calcAngleNew(tamponDesired[6],tamponDesired1[6], tampon[6], tampon1[6], tampon2[6], &torquetest);
    PLOT("module:DynamicController:rHipYawPitchAfter", (desiredJointAngles.angles[Joints::rHipYawPitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:rHipRollBefore",desiredJointAngles.angles[Joints::rHipRoll]);
    desiredJointAngles.angles[Joints::rHipRoll] = calcAngleNew(tamponDesired[7],tamponDesired1[7], tampon[7], tampon1[7], tampon2[7], &torquetest);
    PLOT("module:DynamicController:rHipRollAfter", (desiredJointAngles.angles[Joints::rHipRoll])* (180.0 / M_PI));

    PLOT("module:DynamicController:rHipPitchBefore",desiredJointAngles.angles[Joints::rHipPitch]);
    desiredJointAngles.angles[Joints::rHipPitch] = calcAngleNew(tamponDesired[8],tamponDesired1[8], tampon[8], tampon1[8], tampon2[8], &torquetest);
    PLOT("module:DynamicController:rHipPitchAfter", (desiredJointAngles.angles[Joints::rHipPitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:rKneePitchBefore", desiredJointAngles.angles[Joints::rKneePitch]);
    desiredJointAngles.angles[Joints::rKneePitch] = calcAngleNew(tamponDesired[9],tamponDesired1[9], tampon[9], tampon1[9], tampon2[9], &torquetest);
    PLOT("module:DynamicController:rKneePitchAfter", (desiredJointAngles.angles[Joints::rKneePitch])* (180.0 / M_PI));

    PLOT("module:DynamicController:rAnklePitchBefore", desiredJointAngles.angles[Joints::rAnklePitch]);
    desiredJointAngles.angles[Joints::rAnklePitch] = calcAngleNew(tamponDesired[10],tamponDesired1[10], tampon[10], tampon1[10], tampon2[10],&torquetest);
    PLOT("module:DynamicController:rAnklePitchAfter", (desiredJointAngles.angles[Joints::rAnklePitch])* (180.0 / M_PI));
    
    PLOT("module:DynamicController:rAnkleRollBefore",desiredJointAngles.angles[Joints::rAnkleRoll]);
    desiredJointAngles.angles[Joints::rAnkleRoll] = calcAngleNew(tamponDesired[11],tamponDesired1[11], tampon[11], tampon1[11], tampon2[11],&torquetest);
    PLOT("module:DynamicController:rAnkleRollAfter", (desiredJointAngles.angles[Joints::rAnkleRoll])* (180.0 / M_PI));

    // OUTPUT_TEXT("Angle desiré rAnkleRoll : " << desiredJointAngles.angles[Joints::rAnkleRoll]);
    return true;
}

Angle DynamicController::calcAcceleration(Angle dQ0, Angle dQ1, float delta){ //equation 6
    return (dQ0-dQ1)/delta;
}

Angle DynamicController::calcErrorPosition(Angle Q, Angle Qd){// E(t), primitive de equation 7
    return Q - Qd;
}

Angle DynamicController::calcErrorVelocity(Angle dQ, Angle dQd){//equation 7
    return dQ - dQd;
}

Angle DynamicController::calcSlidingSurface(Angle dQWithError, Angle QWithError){
    return dQWithError + (beta1+(1/(2*theta0))*D*D) * QWithError; //Equation 12, S(t)

}


Angle DynamicController::sign(Angle sigma) { // Equation 17
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


Angle DynamicController::calcTorque(Angle ddQd, Angle dQWithError, Angle QWithError, Angle sigma){// equation 18
       return M * (ddQd - QWithError - (beta1+(1/(2*theta0))*D*D) * dQWithError- beta2*(sigma) - K * calcD(dQWithError, QWithError)*saturation(sigma));
}

void DynamicController::calcTDE(Angle& currentTorque, Angle previousTorque, Angle previousAcceleration) {
    // Calculate the intermediate torque based on the previous time step's data
    Angle intermediateTorque = previousTorque - M * previousAcceleration;

    // Update the current torque with the intermediate torque
    currentTorque += intermediateTorque;
}


Angle DynamicController::calcD(Angle dQWithError, Angle QWithError) {

	return 1 / (float)(li+(1-li)*(exp(-pow(fabs(dQWithError + (beta1+(1/(2*theta0))*D*D) * QWithError),pi))));
}


/*------------------------------------------NEW CONTROLLER-----------------------------------------------------------------------------*/
/*Fonctions utilisees*/
void DynamicController:: verifMaxError(Angle& error)
{
    if(abs(error) > e0)
    {
        error = e0;
    }
}

Angle DynamicController::calcS(Angle epsilonPoint, Angle epsilonLatino)//eq 14
{
    return epsilonPoint + lambda_1 * epsilonLatino+lambda_2 * sig(alpha1, epsilonLatino) + lambda_3 * sig(alpha2, epsilonLatino);
}


Angle DynamicController::calcGammaEqM(Angle ddQd, Angle H_hat)//eq 17 a corriger et modifier
{
    return M*(ddQd - H_hat);
}

Angle DynamicController::calcGammaFtM(Angle s) //eq 19 
{
    return -M*K1*s-M*K2*sig(etat1, s)-M*K3*sig(etat2, s)-M*K4*sign(s);
}

Angle DynamicController::calcGammaM(Angle gammaEqM, Angle gammaFtM)//eq 20
{
    return gammaFtM + gammaEqM;
}


Angle DynamicController::calcThetaCmd(Angle theta, Angle thetaPoint, Angle gammaM)//eq 21
{
    
    return theta + 1/Kp*(Kd*thetaPoint-gammaM);
}

Angle DynamicController::calcEpsilonTilde(Angle fi, Angle epsilon)// peut ressembler a calcErrorPosition(), changer les valeurs entrees de la fonction. Eq 13 du nouveau paper
{
    return 0.5f * logf((fi + (float)(epsilon)) / (fi - (float)(epsilon)));
}

Angle DynamicController :: calcDEpsilonTilde(Angle epsilonTilde)//eq 15
{
    return -lambda_1*epsilonTilde-lambda_2*sig(alpha1,epsilonTilde)-lambda_3*sig(alpha1,epsilonTilde);
}

Angle DynamicController::sig(float exponent, Angle argument)
{
    return powf(fabsf((float)(argument)),(float)(exponent)) * sign((float)(argument));
}


/*Fonction non-utilises*/

Angle DynamicController::calcFi(Angle f0, Angle fInf, double l)//eq 11 conditions t
{
    return (float)(f0 - fInf) * expf(-(float)(l)) + (float)(fInf);

}
/*Fonction principale*/
Angle DynamicController::calcAngleNew(Vector3a desiredAngle0, Vector3a desiredAngle1, Vector2a angle0, Vector2a angle1, Vector2a angle2, Angle* testTorque) {
    // Extract desired angles and their derivatives
    Angle Qd = desiredAngle0[0]; // Desired position
    Angle dQd = desiredAngle0[1]; // Desired velocity
    Angle Q = angle0[0]; // Current position
    Angle dQ = angle0[1]; // Current velocity

    // Calculate acceleration at the previous time step
    Angle ddQ1 = calcAcceleration(angle1[1], angle2[1], delta); //Related to Equation 5

    // Calculate position error
    Angle QWithError = calcErrorPosition(Q, Qd); //Related to Equation 14
    verifMaxError(QWithError);

    // Calculate velocity error
    Angle dQWithError = calcErrorVelocity(dQ, dQd); // Related to Equation 14

    // Calculate sliding surface
    Angle sigma = calcSlidingSurface(dQWithError, QWithError); // Related to Equation 14

    // Calculate desired acceleration
    Angle ddQd = calcAcceleration(dQd, desiredAngle1[1], delta); //Related to Equation 5

    // Calculate and store initial torque in desired angle vector
    desiredAngle0[2] = calcTorque(ddQd, dQWithError, QWithError, sigma); // Related to Equation 17
    *testTorque = desiredAngle0[2];

    // Apply Time-Delay Estimation (TDE) to the current torque
    calcTDE(desiredAngle0[2], desiredAngle1[2], ddQ1); // Related to Equation 5

    // Further calculations for control
    Angle epsilon_dot = calcDEpsilonTilde(QWithError); // related to Equation 14
    Angle s = calcS(epsilon_dot, QWithError); //Equation 14

    // Calculate equivalent control component
    Angle gamma_eq_m = calcGammaEqM(ddQd, desiredAngle0[2]); // Related to Equation 17

    // Calculate fixed-time control component
    Angle gamma_ft_m = calcGammaFtM(s); // Related to Equation 19

    // Calculate total control
    Angle gamma_m = gamma_eq_m + gamma_ft_m; // Related to Equation 20

    // Calculate final command for the angle
    Angle finalCommand = calcThetaCmd(Q, dQ, gamma_m); // Related to Equation 21

    return finalCommand-offset;
}
