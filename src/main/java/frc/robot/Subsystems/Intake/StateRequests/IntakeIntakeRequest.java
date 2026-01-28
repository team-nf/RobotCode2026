package frc.robot.subsystems.Intake.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.constants.States.IntakeStates.IntakeControlState;

public class IntakeIntakeRequest extends InstantCommand {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeIntakeRequest(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeState(IntakeControlState.INTAKE);
  }
}
