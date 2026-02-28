package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.TheMachine.TheMachine;

public class TheMachineGetReadyAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shootRequest(),
        theMachine.feederReverseRequest(),
        theMachine.hopperZeroRequest(),
        theMachine.intakeDeployRequest()
    ).until(() -> (theMachine.getState() != TheMachineControlState.GET_READY));
  }
}