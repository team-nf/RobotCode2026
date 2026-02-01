package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.States.TheMachineStates.TheMachineControlState;

public class TheMachineReverseAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterZeroRequest(),
        theMachine.feederReverseRequest(),
        theMachine.hopperReverseRequest(),
        theMachine.intakeReverseRequest()
    ).until(() -> (theMachine.getState() != TheMachineControlState.REVERSE));
  }
}