/** behavior for the ready state */

option(ReadyState)
{
    initial_state(ready)
    {
        transition
        {
            goto startRoleAttribution;
        }
    }

    state(startRoleAttribution)
    {
        transition
        {
            if(action_done)
                goto sweep;
        }
        action
        {
            if(theLibCodeRelease.lastGameState != STATE_PLAYING)
            {
                if(theRobotInfo.number == 1)
                    theBehaviorStatus.role = Role::keeper;
                else if(theRobotInfo.number == 2)
                    theBehaviorStatus.role = Role::rightDefender;
                else if(theRobotInfo.number == 3)
                    theBehaviorStatus.role = Role::leftDefender;
                else if(theRobotInfo.number == 4)
                    theBehaviorStatus.role = Role::supporter;
                else if(theRobotInfo.number == 5)
                    theBehaviorStatus.role = Role::striker;
            }

            LookForward();
            Stand();
        }
    }

    state(sweep)
    {
        transition
        {
            if (action_done)
                goto moveToStartPosition;
        }
        action
        {
            HeadSweep();
        }
    }

    state(moveToStartPosition)
    {
        transition
        {
            if (state_time > 35000 && theRobotPose.translation.x() < 0)
                goto turnToCenter;
        }
        action
        {
            HeadSweep(100, pi*0.4);
            MovePlayerToStartPosition();
        }
    }

    state(turnToCenter)
    {
        transition
        { 
            if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle()) < 6_deg)
                goto stand;
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(4.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle(), 0.f, 0.f), 1);
        }
    }

    state(stand)
    {
        action
        {
            Stand();
        }
    }
}
