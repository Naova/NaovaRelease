//
// Created by gortium on 02/12/18.
//

#ifndef NAOVACODERELEASE_DRIBBLINGQLEARNINGPROVIDER_H
#define NAOVACODERELEASE_DRIBBLINGQLEARNINGPROVIDER_H

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/DribblingQLearning.h"
//#include "Representations/BehaviorControl/QTrainer.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include <map>
#include <vector>
#include <random>
#include <string.h>
#include <algorithm>
#include "Tools/Math/Geometry.h"
#include <iostream>
#include <fstream>
#include <math.h>


MODULE(DribblingQLearningProvider,
{,
        REQUIRES(TeamBallModel),
        REQUIRES(FieldDimensions),
        REQUIRES(RobotPose),
        REQUIRES(FrameInfo),
        REQUIRES(GameInfo),
       // USES(QTrainer),
        USES(BehaviorStatus),
        PROVIDES(DribblingQLearning),
});

struct State
{
    Vector2f ballTrans;
    Vector2f targetTrans;
    Pose2f robotPos;
};

struct Action
{
    std::string rAction;
    std::string xAction;
    std::string yAction;
};

typedef std::map<std::string, float> Features;
typedef std::map<std::string, float> Weights;

class DribblingQLearningProvider : public DribblingQLearningProviderBase
{
    bool learningActivated = false;

    State state;
    State prevState;
    Action prevAction;
    Action action;
    Features features;
    Weights weights;
    float cumulativeReward = 0.0;
    //bool learningActivated = false;
    unsigned int lastTime = 0;
    unsigned int period = 1000;

    float Vr = 0.0;
    float Vx = 0.0;
    float Vy = 0.0;

    float epsilon = 0.2f;
    const float discount = 0.3f;
    const float alpha = 0.1f;

    float lastRho = 0;
    float lastPsi = 0;

    const std::vector< std::string > PossibleRActions{"Higher", "Lower", "Same"};
    const std::vector< std::string > PossibleXActions{"Higher", "Lower", "Same"};
    const std::vector< std::string > PossibleYActions{"Higher", "Lower", "Same"};

    std::vector< Action > legalActions; // Storing it to reduce computing and not check for legal action every step

    void update(DribblingQLearning& qlearning);
    void updateState(State& state, State& prevState);
    float getReward(State& state);
    void updateWeights(State& state, State& prevState);
    std::vector< Action > getLegalAction(State& state);
    Features getFeatures(State& state, Action& action);
    float getQValue(State& state, Action& action);
    float computeValueFromQValues(State& state);
    Action computeActionFromQValues(State &state);
    Action getAction(State& state);
    void applyAction(Action& action, DribblingQLearning& qlearning);
    void saveWeights();
    void saveResults();
    float updateSpeed(std::string action, float speed);
};

#endif //NAOVACODERELEASE_DRIBBLINGQLEARNINGPROVIDER_H
