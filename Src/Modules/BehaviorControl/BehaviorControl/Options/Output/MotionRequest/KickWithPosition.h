option(KickWithPosition, (const Vector2f&) position)
{
    initial_state(align)
    {
        transition
        {
            if(action_done)
            {
                goto kick;
            }
        }
        action
        {
            AlignForKick((theRobotPose.inversePose * position).angle());
        }
    }

    state(kick)
    {
        transition
        {
            if(action_done)
            {
               goto ballkicked;
            }
        }

        action
        {
            if(theRobotPose.translation.x() + theBallModel.estimate.position.x() >= theFieldDimensions.xPosOpponentGroundline - theLibCodeRelease.kickFarRange)
            {
                KickFar((theRobotPose.inversePose * position).angle());    
            }
            else
            {
               InWalkKick(WalkKickVariant(WalkKicks::forwardShoot, Legs::left),
               Pose2f((theRobotPose.inversePose * position).angle(),theRobotPose.inversePose * position));
            }
        }
    }

    target_state(ballkicked){}
}