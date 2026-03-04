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

    Command commandWithShake = new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(
            theMachine.feederFeedRequest(),
                    (theMachine.hopperReverseRequest()
                      .andThen(new WaitCommand(0.2))).unless(theMachine::checkHopperPrevFeed),

                      (theMachine.hopperFeedRequest()))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(0.75))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(3)));

      Command commandWithShakeHeavyDuty = new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(new WaitCommand(0.1))
        .andThen(
            theMachine.feederFeedRequest(),
                    theMachine.hopperReverseRequest()
                      .andThen(new WaitCommand(0.1))
                      .andThen(theMachine.hopperFeedRequest()))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(1))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(0.5)))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(1))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(0.5));

      Command passCommand = new ParallelCommandGroup(
        theMachine.shooterTestRequest(),
        (new WaitCommand(0.2))
        .andThen(
            theMachine.feederFeedRequest(),
                    theMachine.hopperReverseRequest()
                      .andThen(new WaitCommand(0.1))
                      .andThen(theMachine.hopperFeedRequest()))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(1))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(0.5)))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(1))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(0.5));

    return commandWithShake.until(() -> (theMachine.getState() != TheMachineControlState.SHOOT));

    /*return new ConditionalCommand
        (commandWithShake, commandWithShake, theMachine::isThereLotsOfFuel)
      .until(() -> (theMachine.getState() != TheMachineControlState.SHOOT));*/
  }
}