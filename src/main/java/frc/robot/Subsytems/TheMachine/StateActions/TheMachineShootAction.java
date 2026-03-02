package frc.robot.Subsytems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.TheMachine.TheMachine;

public class TheMachineShootAction {

  public static Command get(TheMachine theMachine) {

    Command commandToRunWithLessFuel = new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(new WaitCommand(0.1))
        .andThen(
            theMachine.hopperFeedRequest(),
            theMachine.feederFeedRequest())
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(3)));

      Command commandToRunWithManyFuel = new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(new WaitCommand(0.1))
        .andThen(
            theMachine.hopperFeedRequest(),
            theMachine.feederFeedRequest())
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(3))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(3)));

    return new ConditionalCommand
        (commandToRunWithManyFuel, commandToRunWithLessFuel, theMachine::isThereLotsOfFuel)
      .until(() -> (theMachine.getState() != TheMachineControlState.SHOOT));
  }
}