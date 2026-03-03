package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.TheMachine.TheMachine;

public class TheMachineTestAction {

  public static Command get(TheMachine theMachine) {
    Command commandWithShake = new ParallelCommandGroup(
        theMachine.shooterTestRequest(),
        theMachine.waitForShooter()
        .andThen(new WaitCommand(0.2))
        .andThen(
            theMachine.feederFeedRequest(),
                    theMachine.hopperReverseRequest()
                      .andThen(new WaitCommand(0.1))
                      .andThen(theMachine.hopperFeedRequest()))
        .andThen(theMachine.intakeIntakeRequest()));

    return commandWithShake.until(() -> (theMachine.getState() != TheMachineControlState.TEST));
  }
}