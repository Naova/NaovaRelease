option(WalkToBall, (Vector2f) kickPos)
{
    common_transition
    {
        // finish once close enough for align
        if(theBallModel.estimate.position.norm() < 300.f || (theBallModel.estimate.position.norm() < 400.f && std::abs(theRobotPose.rotation) < 90_deg && std::abs((theRobotPose.inversePose * kickPos).angle()) < 3_deg))
        {
            goto finished;
        }
        // check if the ball is on the opposite side of the goal
        if(std::abs(theRobotPose.rotation) < 90_deg)
        {
            if(theBallModel.estimate.position.norm() >= 400.f)
            {
                goto walkDirect;
            }
            else
            {
                goto walkDirectCloser;
            }
        }
    }

    initial_state(reposition)
    {
        transition
        {
            if(std::abs(theRobotPose.rotation) >= 90_deg) 
            {
                if(theBallModel.estimate.position.norm() >= 400.f)
                {
                    goto walkToSide;
                }
                else
                {
                    goto walkCloser;
                }
            }
        }
    }

    state(walkToSide)
    {
        transition
        {
            // check if the ball is on the opposite side of the goal
            if(std::abs(theRobotPose.rotation) >= 90_deg)
            {
                // wait until we're somewhat facing the ball before choosing a side
                if(theBallModel.estimate.position.norm() >= 400.f && std::abs(theBallModel.estimate.position.angle()) < 60_deg)
                {
                    // choose left or right based on our position in relation to the ball
                     if(std::abs(theBallModel.estimate.position.angle()) <= 90_deg)
                    {
                        if(theRobotPose.rotation >= 0) //left
                        {
                            goto walkLeft;
                        }
                        else //right
                        {
                            goto walkRight;
                        }
                    }
                    else
                    {
                        if(theRobotPose.rotation >= 0) //right
                        {
                            goto walkRight;
                        }
                        else //left
                        {
                            goto walkLeft;
                        }
                    }
                }
                else if(theBallModel.estimate.position.norm() < 400.f)
                {
                    goto walkCloser;
                }
            }
        }
        action
        {
            float angleOffset = 0.f;                                                                                                                                                                         
            float xOffset = 200.f;
            float yOffset = 300.f;

            // choose left or right based on our position in relation to the ball
            if(std::abs(theBallModel.estimate.position.angle()) <= 90_deg)
            {
                if(theRobotPose.rotation >= 0) //left
                {
                    angleOffset = std::abs(atanf((theBallModel.estimate.position.y() + yOffset) / (theBallModel.estimate.position.x() - xOffset)));
                }
                else //right
                {
                    yOffset = -yOffset;
                    angleOffset = -std::abs(atanf((theBallModel.estimate.position.y() + yOffset) / (theBallModel.estimate.position.x() - xOffset)));
                }
            }
            else
            {
                if(theRobotPose.rotation >= 0) //right
                {
                    yOffset = -yOffset;
                    angleOffset = -std::abs(atanf((theBallModel.estimate.position.y() + yOffset) / (theBallModel.estimate.position.x() - xOffset)));
                }
                else //left
                {
                    angleOffset = std::abs(atanf((theBallModel.estimate.position.y() + yOffset) / (theBallModel.estimate.position.x() - xOffset)));
                }
            }

            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle() + angleOffset, theBallModel.estimate.position.x() - xOffset, theBallModel.estimate.position.y() + yOffset), 1);
        }

    }

    state(walkLeft)
    {
        transition
        {
            // choose left or right based on our position in relation to the ball
            if(std::abs(theRobotPose.rotation) >= 90_deg)
            {
                if(theBallModel.estimate.position.norm() >= 400.f && state_time > 2000.f)
                {
                     if(std::abs(theBallModel.estimate.position.angle()) <= 90_deg)
                    {
                        if(theRobotPose.rotation >= 0) //left
                        {
                            goto walkLeft;
                        }
                        else //right
                        {
                            goto walkRight;
                        }
                    }
                    else
                    {
                        if(theRobotPose.rotation >= 0) //right
                        {
                            goto walkRight;
                        }
                        else //left
                        {
                            goto walkLeft;
                        }
                    }
                }
                else if(theBallModel.estimate.position.norm() < 400.f)
                {
                    goto walkCloser;
                }
            }
        }
        action
        {
            float angleOffset = 0.f;                                                                                                                                                                         
            float xOffset = 200.f;
            float yOffset = 300.f;

            angleOffset = std::abs(atanf((theBallModel.estimate.position.y() + yOffset) / (theBallModel.estimate.position.x() - xOffset)));

            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle() + angleOffset, theBallModel.estimate.position.x() - xOffset, theBallModel.estimate.position.y() + yOffset), 1);
        }
    }

    state(walkRight)
    {
        transition
        {
            // choose left or right based on our position in relation to the ball
            if(std::abs(theRobotPose.rotation) >= 90_deg)
            {
                if(theBallModel.estimate.position.norm() >= 400.f && state_time > 2000.f)
                {
                     if(std::abs(theBallModel.estimate.position.angle()) <= 90_deg)
                    {
                        if(theRobotPose.rotation >= 0) //left
                        {
                            goto walkLeft;
                        }
                        else //right
                        {
                            goto walkRight;
                        }
                    }
                    else
                    {
                        if(theRobotPose.rotation >= 0) //right
                        {
                            goto walkRight;
                        }
                        else //left
                        {
                            goto walkLeft;
                        }
                    }
                }
                else if(theBallModel.estimate.position.norm() < 400.f)
                {
                    goto walkCloser;
                }
            }
        }
        action
        {
            float angleOffset = 0.f;                                                                                                                                                                         
            float xOffset = 200.f;
            float yOffset = 300.f;

            angleOffset = std::abs(atanf((theBallModel.estimate.position.y() - yOffset) / (theBallModel.estimate.position.x() - xOffset)));

            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle() - angleOffset, theBallModel.estimate.position.x() - xOffset, theBallModel.estimate.position.y() - yOffset), 1);
        }
    }

    state(walkCloser)
    {
        transition
        {
            if(std::abs(theRobotPose.rotation) >= 90_deg)
            {
                if(theBallModel.estimate.position.norm() >= 400.f)
                {
                    goto walkToSide;
                }
            }
        }
        action
        {
            float xOffset = 200.f;
            float yOffset = 300.f;

            // choose left or right based on our position in relation to the ball
            if(std::abs(theBallModel.estimate.position.angle()) <= 90_deg)
            {
                if (theRobotPose.rotation < 0)
                {
                    yOffset = -yOffset;
                }
            }
            else
            {
                if(theRobotPose.rotation >= 0)
                {
                    yOffset = -yOffset;
                }
            }

            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position.x() - xOffset, theBallModel.estimate.position.y() + yOffset));
        }

    }

    state(walkDirect)
    {
        transition
        {
            if(std::abs(theRobotPose.rotation) >= 90_deg)
            {
                if(theBallModel.estimate.position.norm() >= 400.f)
                {
                    goto walkToSide;
                }
                else
                {
                    goto walkCloser;
                }
            }
        }
        action
        {
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position), 1);
        }
    }

    state(walkDirectCloser)
    {
        transition
        {
            if(std::abs(theRobotPose.rotation) >= 90_deg)
            {
                if(theBallModel.estimate.position.norm() >= 400.f)
                {
                    goto walkToSide;
                }
                else
                {
                    goto walkCloser;
                }
            }
        }
        action
        {
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position), 1);
        }
    }

    target_state(finished){}
}