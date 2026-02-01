package frc.robot.Subsytems.Intake.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Intake.IntakeSubsystem;
import frc.robot.Utils.States.IntakeStates.IntakeControlState;

public class IntakeTestAction extends Command {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeTestAction(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.test();
  }

  @Override
  public void execute() {
    intakeSubsystem.test();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !intakeSubsystem.isIntakeState(IntakeControlState.TEST);
  }
}
