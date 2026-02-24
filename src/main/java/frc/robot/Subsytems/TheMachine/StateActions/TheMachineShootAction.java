package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.States.TheMachineStates.TheMachineControlState;

public class TheMachineShootAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(new WaitCommand(0.5))
        .andThen(
            theMachine.hopperFeedRequest(),
            theMachine.feederFeedRequest(),
            theMachine.intakeFeedRequest()
        )
    ).until(() -> (theMachine.getState() != TheMachineControlState.SHOOT));
  }
}