package frc.robot.Subsytems.Intake.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Intake.IntakeSubsystem;
import frc.robot.Utils.States.IntakeStates.IntakeControlState;

public class IntakeIntakeAction extends Command {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeIntakeAction(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.intake();
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !intakeSubsystem.isIntakeState(IntakeControlState.INTAKE);
  }
}
