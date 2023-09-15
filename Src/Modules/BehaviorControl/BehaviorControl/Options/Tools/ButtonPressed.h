/**
 * The option observes the state of a certain button and reports a success if it was pressed and released.
 * If a timeout is specified, the option can also be aborted if state changes take too long.
 * @param key The button to observe.
 * @param timeOut An optional timeout in ms. The timeout is measured between state changes.
 */
option(ButtonPressed, ((KeyStates) Key) key)
{
  /** The option waits for the button to be pressed. */
  initial_state(waitingForPressInit)
  {
    transition
    {
      
      if(theKeyStates.pressed[key])
        goto success;
    }
  }

  /** Button was pressed and released. The option only stays a single cycle in this state. */
  target_state(success)
  {
    transition
    {
    }
  }
}
