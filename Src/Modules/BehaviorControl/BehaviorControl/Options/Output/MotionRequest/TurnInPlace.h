/*  Option to turn in place at a specified speed for a target angle
    Input a positive speed to turn counter-clockwise and a negative speed to turn clockwise
    The target can be more than 360 degrees
*/
Angle trgt;
int counter;
int halfTurns;

option(TurnInPlace, (float) (0.5f) speed, (float) (0.f) target)
{
    initial_state(chooseTurn)
    {
        transition
        {
            //if the target is not define It will rotate indefinitely counterclockwise
            if(target == 0.f) 
                goto infTurn;
            else if(target > 180_deg) 
                goto startTurn;
            else
                goto setTarget; 
        }
        action
        {
            counter = 0;
            //every turn is define as 180 deg
            halfTurns = (int)floor(target / 180_deg);
        }
    }

    state(increment)
    {
        transition
        {
            //increments the turns counter 
            if(counter > halfTurns)
                goto setTarget;
            else
                goto startTurn;
        }
        action
        {
            counter++;
            halfTurns = (int)floor(target / 180_deg);
        }
    }

    state(startTurn)
    {
        transition
        {
            if(counter >= halfTurns)
            {
                goto setTarget;
            }
            else
                goto make180;
        }
        action
        {
            halfTurns = (int)floor(target / 180_deg);

            if(speed >= 0)
                //positive speed:  the robot will rotate to the left 
                trgt = Angle(theRobotPose.rotation + 180_deg).normalize();  
            else
                //negative speed:  the robot  will rotate to the right
                trgt = Angle(theRobotPose.rotation - 180_deg).normalize();  
        }
    }

    state(make180)
    {
        // makes a 180 turn 
        transition
        {
            if(theRobotPose.rotation.diffAbs(trgt) < 6_deg)  
                goto increment;
        }
        action
        {
            halfTurns = (int)floor(target / 180_deg);
            WalkAtRelativeSpeed(Pose2f(speed, 0.f, 0.f));
        }
    }

    state(setTarget)
    {
        //decides the angle for the last turn depending on the parameter "speed" (rigth or left)
        transition
        {
            goto turn;
        }
        action
        {
            if(target > 180_deg)
            {
                if(speed >= 0)
                    trgt = Angle(theRobotPose.rotation + (target - halfTurns * 180_deg)).normalize();
                else
                    trgt = Angle(theRobotPose.rotation - (target - halfTurns * 180_deg)).normalize();
            }
            else
            {
                if(speed >= 0)
                    trgt = Angle(theRobotPose.rotation + target).normalize();
                else
                    trgt = Angle(theRobotPose.rotation - target).normalize();
            }
        }
    }


    state(turn)
    {
        //makes the last turn 
        transition
        {
            if(theRobotPose.rotation.diffAbs(trgt) < 10_deg)
                goto done;
        }
        action
        {
            WalkAtRelativeSpeed(Pose2f(speed, 0.f, 0.f));
        }
    }

    target_state(done)
    {
        transition
        {
            goto chooseTurn;
        }
        action
        {
            Stand();
        }
    }

    state(infTurn)
    {
        action
        {
            WalkAtRelativeSpeed(Pose2f(speed, 0.f, 0.f));
        }
    }
}