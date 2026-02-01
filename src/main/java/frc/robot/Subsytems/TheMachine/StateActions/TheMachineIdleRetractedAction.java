package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.States.TheMachineStates.TheMachineControlState;

public class TheMachineIdleRetractedAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterRestRequest(),
        theMachine.feederZeroRequest(),
        theMachine.hopperZeroRequest(),
        theMachine.intakeCloseRequest()
    ).until(() -> (theMachine.getState() != TheMachineControlState.IDLE_RETRACTED));
  }
}