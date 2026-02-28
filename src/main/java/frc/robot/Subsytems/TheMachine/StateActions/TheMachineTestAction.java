package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.TheMachine.TheMachine;

public class TheMachineTestAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterTestRequest(),
        theMachine.feederTestRequest(),
        theMachine.hopperTestRequest(),
        theMachine.intakeTestRequest()
    ).until(() -> (theMachine.getState() != TheMachineControlState.TEST));
  }
}