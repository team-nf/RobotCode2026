package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.TheMachine.TheMachine;

public class TheMachineShootAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(new WaitCommand(0.1))
        .andThen(
            theMachine.hopperFeedRequest(),
            theMachine.feederFeedRequest())
        .andThen(new WaitCommand(0.5))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(1.5))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(3))
    ).until(() -> (theMachine.getState() != TheMachineControlState.SHOOT));
  }
}