/** Fichier gerant les etapes d'un Ground punch sur la Gauche  */
option(GroundPunchLeft)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::groundPunchLeft)
        goto requestLeftIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::groundPunchLeft;
    }
  }


  /** The motion process has started executing the request. */
  target_state(requestLeftIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion =! MotionRequest::groundPunchLeft)
        goto setRequest;
    }
    action
    {
      GetUpEngine();
      LookForward();
      
    }
  }
}
