//
// Created by gortium on 02/12/18.
//

#include "DribblingQLearningProvider.h"

MAKE_MODULE(DribblingQLearningProvider, behaviorControl);

void DribblingQLearningProvider::update(DribblingQLearning& qlearning)
{

    if(this->lastTime + this->period < theFrameInfo.time && (theGameInfo.getStateAsString() == "Playing" || theGameInfo.getStateAsString() == "Finished" || theGameInfo.getStateAsString() == "Unknown")) //&& this->learningActivated
    {
        if(learningActivated)
        {
            updateState(this->state, this->prevState);

            updateWeights(this->state, this->prevState);
        }

        this->action = getAction(this->state);

        applyAction(this->action, qlearning);

        this->lastTime = theFrameInfo.time;

       //printf ("******************DONE****************** \n\n\n\n\n");
    }
    else
    {
        this->cumulativeReward = 0.0;
        updateState(this->state, this->prevState);
    }
}

void DribblingQLearningProvider::updateState(State& state, State& prevState)
{
    prevState = state;
    state.ballTrans = theTeamBallModel.position;
    state.robotPos = theRobotPose;
    state.targetTrans = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
   // printf ("updateState() \n");
}

float DribblingQLearningProvider::getReward(State& state)
{
    float rho = Geometry::distance(state.robotPos.translation, state.ballTrans);
    float psi = Geometry::distance(state.ballTrans, state.targetTrans);

    const float rhoK = 30000.0;
    const float psiK = 50000.0;

    // Inversement proportionnel à la distance + proportionnel a la difference entre la derniere valeur et l'actuel
    float retval = ((rhoK/rho) + (psiK/psi)) + ((lastRho - rho)*10 + (lastPsi - psi)*10);
    // (rhoK/rho) + (psiK/psi) <--- Pour inversement proportionnel à la distance
    // diff --> (lastRho - rho) + (lastPsi - psi)

    lastRho = rho;
    lastPsi = psi;

    // printf ("getReward() \n");

    return retval;
}

void DribblingQLearningProvider::updateWeights(State& state, State& prevState)
{
    Features features = getFeatures(state, action);
    float reward = getReward(state);

    //printf ("REWARD \t\t\t = \t %f \n", reward);

    this->cumulativeReward += reward;

    float d = (reward + this->discount * computeValueFromQValues(state)) - computeValueFromQValues(prevState);

    //printf ("STATE Q \t\t\t = \t %f \n", computeValueFromQValues(state));
    //printf ("PREVSTATE Q \t\t\t = \t %f \n", computeValueFromQValues(prevState));
    //printf ("D \t\t\t = \t %f \n", d);

    for(std::map<std::string, float>::iterator it = this->weights.begin(); it != this->weights.end(); ++it)
    {
        it->second = it->second + this->alpha * d * (features)[it->first];
        //printf ("WEIGHT  \t %s \t = \t %f \n", it->first.c_str(), it->second);
        //printf ("FEATURE \t %s \t = \t %f \n", it->first.c_str(), (features)[it->first]);
    }
   // printf ("updateWeights() \n");
   //printf ("Alpha \t = \t %f \n", this->alpha);
   //printf ("Epsilon \t = \t %f \n", this->epsilon);
   //printf ("Discount \t = \t %f \n", this->discount);

}

std::vector< Action > DribblingQLearningProvider::getLegalAction(State& state)
{
    Action action;
    std::vector< Action > legalActions;

    for(int r = 0; r < (int)this->PossibleRActions.size(); r++)
    {
        for(int x = 0; x < (int)this->PossibleXActions.size(); x++)
        {
            for(int y = 0; y < (int)this->PossibleYActions.size(); y++)
            {
                action.rAction = this->PossibleRActions[r];
                action.xAction = this->PossibleXActions[x];
                action.yAction = this->PossibleYActions[y];

                legalActions.push_back(action);
            }
        }
    }
    //printf ("getLegalAction() \n");
    return legalActions;
};

float DribblingQLearningProvider::updateSpeed(std::string action, float speed){
    //TODO put global variable for changing speed


    if(action == "Higher")
    {
        speed += 25;
    }
    else if(action == "Lower")
    {
        speed -= 25;
    }
    return speed;

}

Features DribblingQLearningProvider::getFeatures(State& state, Action& action)
{
    Features features;

    float Vr = updateSpeed(action.rAction, this->Vr) * 0.45f;
    float Vx = updateSpeed(action.xAction, this->Vx);
    float Vy = updateSpeed(action.yAction, this->Vy);

    float deltaTimeSec = theFrameInfo.getTimeSince(this->lastTime) * 0.001f;

    float delta_x = (Vx * (float)cos(state.robotPos.rotation) - Vy * (float)sin(state.robotPos.rotation))* deltaTimeSec;
    float delta_y = (Vx * (float)sin(state.robotPos.rotation) + Vy * (float)cos(state.robotPos.rotation))* deltaTimeSec;
    float delta_angle = Vr * deltaTimeSec;

    Pose2f robot_next_pose(state.robotPos.rotation + delta_angle, state.robotPos.translation.x() + delta_x, state.robotPos.translation.y() + delta_y);

    float dist_player_ball = Geometry::distance(robot_next_pose.translation, state.ballTrans);
    float angle_gamma = Geometry::angleTo(robot_next_pose, state.targetTrans);
    float angle_alpha = Geometry::angleTo(robot_next_pose, state.ballTrans);

    Pose2f ballFacingTarget(0.0, state.ballTrans.x(), state.ballTrans.y());
    float ballAngleToRobot = Geometry::angleTo(ballFacingTarget, Vector2f(robot_next_pose.translation.x(), robot_next_pose.translation.y()));
    
    //float dist_player_target = Geometry::distance(robot_next_pose.translation, state.targetTrans);
    //float dist_ball_target = Geometry::distance(state.ballTrans, state.targetTrans);

    //printf ("---> dist_player_target \t\t\t = \t %f \n", dist_player_target);
    //printf ("---> dist_ball_target \t\t\t = \t %f \n", dist_ball_target);
    //printf ("---> angle_alpha \t\t\t = \t %f \n", angle_alpha);
    //printf ("---> angle_gamma \t\t\t = \t %f \n", angle_gamma);
    //printf ("---> sin \t\t\t = \t %f \n", sin((angle_alpha + angle_gamma)));
    //printf ("---> asin \t\t\t = \t %f \n", (float)asin(dist_player_target/dist_ball_target * sin((angle_alpha + angle_gamma))));
    float angle_phi = 3.1416f*0.5f - ballAngleToRobot;// (float)asin(dist_player_target/dist_ball_target * sin((angle_alpha + angle_gamma)));
    //printf ("---> PHI \t\t\t = \t %f \n", angle_phi);

    // NORMALIZATION
    float maxDist = (float)sqrt(pow(theFieldDimensions.xPosOpponentGroundline, 2) + pow(theFieldDimensions.yPosLeftSideline, 2));
    dist_player_ball = (dist_player_ball) / (maxDist);
    angle_alpha = (angle_alpha - -3.1416f) / (3.1416f - -3.1416f);
    angle_gamma = (angle_gamma - -3.1416f) / (3.1416f - -3.1416f);
    angle_phi = (angle_phi - (-3.1416f/2.0f)) / ((3.1416f/2.0f) - (-3.1416f/2.0f));
    Vr = Vr/100.f;
    Vx = Vx/100.f;
    Vy = Vy/100.f;

    features.clear();

    features.insert( std::pair< std::string, float>("distance_player_ball",dist_player_ball));
    features.insert( std::pair< std::string, float>("angle_alpha",angle_alpha));
//    if(dist_player_ball < 0.10)
//    {
        features.insert( std::pair< std::string, float>("angle_gamma",angle_gamma));
//    }
//    if(dist_player_ball < 0.20)
//    {
        features.insert( std::pair< std::string, float>("angle_phi",angle_phi));
//    }
    features.insert( std::pair< std::string, float>("bias", 1));

    //features.insert( std::pair< std::string, float>("velR", Vr));
    //features.insert( std::pair< std::string, float>("velX", Vx));
    //features.insert( std::pair< std::string, float>("velY", Vy));

    //printf ("getFeatures() \n");

    return features;
}

float DribblingQLearningProvider::getQValue(State& state, Action& action)
{
    Features features = getFeatures(state, action);

   // printf ("features size = %i \n", features.size());

    if(this->weights.empty())
    {
        for(std::map<std::string, float>::iterator it = features.begin(); it != features.end(); ++it)
        {
            this->weights.insert(this->weights.begin(), std::pair<std::string, float>(it->first, 1.0));
        }
    }

    //printf ("weights size = %i \n", weights.size());

    float q = 0.0f;
    for(std::map<std::string, float>::iterator it = this->weights.begin(); it != this->weights.end(); ++it)
    {
        q += it->second * (features)[it->first];
        //printf("feature %s = %f \n",it->first.c_str() , (features)[it->first]);
        //printf ("weight %s = %f \n", it->first.c_str(), it->second);
        //printf ("q = %f \t", q);
    }

    //printf ("getQValue() \n");

    return q;
}

float DribblingQLearningProvider::computeValueFromQValues(State& state)
{
    float maxVal = std::numeric_limits<float>::lowest();
    float q = 0.0f;

    std::vector< Action > legalActions = getLegalAction(state);

    for(int i = 0; i < (int)legalActions.size(); i++)
    {
        q = getQValue(state, legalActions[i]);
        if(maxVal <= q)
        {
            maxVal = q;
        }
    }

    //printf ("computeValueFromQValues() \n");

    return maxVal;
}

Action DribblingQLearningProvider::computeActionFromQValues(State &state)
{
    float maxVal = std::numeric_limits<float>::lowest();
    std::vector< Action > maxAction = getLegalAction(state);
    float q = 0.0f;
    std::vector< Action > legalActions = getLegalAction(state);

    //printf ("legalActions size = %i \n", legalActions.size());

    for(int i = 0; i < (int)legalActions.size(); i++)
    {
        q = getQValue(state, legalActions[i]);

        //printf ("q value = %f \n", q);
        //printf ("maxVal = %f \n", maxVal);

        if(maxVal == q)
        {
            //printf (" q egale \n");
            maxAction.push_back(legalActions[i]);
        }

        else if(maxVal < q)
        {
            //printf (" q plus grand \n");
            maxVal = q;
            maxAction.clear();
            maxAction.push_back(legalActions[i]);
        }
        else
        {
            //printf (" q plus petit \n");
        }
    }

    //printf ("computeActionFromQValues() \n");

    //printf ("maxAction size = %i \n", maxAction.size());

    return maxAction[rand() % maxAction.size()];
}

Action DribblingQLearningProvider::getAction(State& state)
{
    std::vector< Action > legalActions = getLegalAction(state);

    this->epsilon = this->epsilon + 0.001f;

    if((float)((rand()) / (float(RAND_MAX) + 1.0)) >= this->epsilon)
    {
       //printf ("getAction() -> random \n");
        return legalActions[rand() % legalActions.size()];
    }
    else
    {
       //printf ("getAction() -> policy \n");
        return computeActionFromQValues(state);
    }
}

void DribblingQLearningProvider::applyAction(Action& action, DribblingQLearning& qlearning)
{
    float step = 25.f;
    if(action.rAction == "Higher")
    {
        //this->Vr = (float)(std::max(-100.0, std::min(this->Vr + step, 100.0)));
        this->Vr = this->Vr + step * ((100.f - this->Vr)/(200.f));
    }
    else if(action.rAction == "Lower")
    {
        //this->Vr = (float)(std::min(100.0, std::max(this->Vr - step, -100.0)));
        this->Vr = this->Vr - step * ((this->Vr - -100.f)/200.f);
    }
    //printf ("applyAction() -> Vr = %f \n", this->Vr);


    if(action.xAction == "Higher")
    {
        //this->Vx = (float)(std::max(-100.0, std::min(this->Vx + step, 100.0)));
        this->Vx = this->Vx + step * ((100.f - this->Vx)/(200.f));
    }
    else if(action.xAction == "Lower")
    {
        //this->Vx = (float)(std::min(100.0, std::max(this->Vx - step, -100.0)));
        this->Vx = this->Vx - step * ((this->Vx - -100.f)/200.f);
    }
   //printf ("applyAction() -> Vx = %f \n", this->Vx);

    if(action.yAction == "Higher")
    {
        //this->Vy = (float)(std::max(-100.0, std::min(this->Vy + step, 100.0)));
        this->Vy = this->Vy + step * ((100.f - this->Vy)/(200.f));

    }
    else if(action.yAction == "Lower")
    {
        //this->Vy = (float)(std::min(100.0, std::max(this->Vy - step, -100.0)));
        this->Vy = this->Vy - step * ((this->Vy - -100.f)/200.f);

    }
   //printf ("applyAction() -> Vy = %f \n", this->Vy);


    qlearning.Vr = this->Vr;
    qlearning.Vx = this->Vx;
    qlearning.Vy = this->Vy;
}

void DribblingQLearningProvider::saveWeights()
{
    static bool first = true;
    std::ofstream myfile;
    myfile.open ("weights.csv");
    if(first == true)
    {
        for(std::map<std::string, float>::iterator it = weights.begin(); it != weights.end(); ++it)
        {
            myfile << it->first << ",";
        }
        myfile << "\n";
        first = false;
    }

    for(std::map<std::string, float>::iterator it = weights.begin(); it != weights.end(); ++it)
    {
        myfile << it->second << ",";
    }
    myfile << "\n";
    myfile.close();
}

void DribblingQLearningProvider::saveResults()
{
    static bool first = true;
    std::ofstream myfile;
    myfile.open ("Results.csv");
    if(first == true)
    {
        myfile << "Cumulative reward,";
        myfile << "\n";
        first = false;
    }

    myfile << this->cumulativeReward <<",";
    myfile << "\n";
    myfile.close();
}
