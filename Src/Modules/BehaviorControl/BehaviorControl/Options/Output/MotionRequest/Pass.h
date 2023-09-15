option(Pass, (const Teammate&) teammate, (const float) maxOpponentBallDistance)
{

    Vector2f passOffset = Vector2f(500.f, 0.f);
    Vector2f supporterPosition = (theRobotPose.inversePose * (teammate.theRobotPose.translation + passOffset));
    Vector2f ballPosition = theRobotPose.inversePose * theTeamBallModel.position;
    Vector2f ballToSupporter = supporterPosition - ballPosition;
    Vector2f positionBehindBall = ballToSupporter.normalized() * (-200.f) + ballPosition;
    Pose2f position = Pose2f(ballPosition.angle(), positionBehindBall.x(), positionBehindBall.y());

    bool opponentIsTooClose = false;

    for(auto const &obstacle : theObstacleModel.obstacles)
    {
        if(obstacle.isOpponent()) 
        {
            float const distanceOfOpponentToBall = (ballPosition - obstacle.center).norm();
            if(distanceOfOpponentToBall < maxOpponentBallDistance) 
            {
                opponentIsTooClose = true;
                break;
            }
        }
    }


    initial_state(setRequest)
    {

        transition
        {
            goto alignForPass;
        }

    }

    target_state(requestIsExecuted){}

    aborted_state(abortedState){}

    state(alignForPass)
    {

        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto abortedState;
            }

            if(opponentIsTooClose && std::abs(supporterPosition.angle()) < 5_deg && positionBehindBall.norm() < 300.f) {
                goto inWalkKick;
            }

            if(positionBehindBall.norm() < 25.f && std::abs(supporterPosition.angle()) < 5_deg)
            {
                goto kick;
            }

            if(ballPosition.norm() > 600.f || !theLibCodeRelease.ballIsInGoalArea)
            {
                goto abortedState;
            }
        }

        action
        {
            LookForward();
            WalkToTarget(Pose2f(80.f, 80.f, 80.f), position);
        }

    }

    state(kick)
    {

        transition
        {
            if(state_time > 10 && action_done)
            {
                goto setRequest;
            }
            if(ballPosition.norm() > 600.f)
            {
                goto requestIsExecuted;
            }
        }

        action
        {
            LookForward();
            KickFar(supporterPosition.angle());
        }

    }

    state(inWalkKick)
    {

        transition
        {
            if(state_time > 10 && action_done)
            {
                goto setRequest;
            }
            if(ballPosition.norm() > 600.f)
            {
                goto requestIsExecuted;
            }
        }

        action
        {
            LookForward();
            auto const leg = ballPosition.y() < 0 ? Legs::right : Legs::left;
            InWalkKick(WalkKickVariant(WalkKicks::forward, leg), 
            Pose2f(supporterPosition.angle(), ballPosition.x() - 160.f, ballPosition.y() - (leg == Legs::left ? 55.f : -55.f)));
        }

    }

}