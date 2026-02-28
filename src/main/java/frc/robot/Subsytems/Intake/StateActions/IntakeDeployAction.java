package frc.robot.Subsytems.Intake.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Subsytems.Intake.IntakeSubsystem;

public class IntakeDeployAction extends Command {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeDeployAction(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.deploy();
  }

  @Override
  public void execute() {
    intakeSubsystem.deploy();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !intakeSubsystem.isIntakeState(IntakeControlState.DEPLOY);
  }
}
