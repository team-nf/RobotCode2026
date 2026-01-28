package frc.robot.subsystems.Intake.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.States.IntakeStates.IntakeControlState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCloseRequest extends InstantCommand {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeCloseRequest(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeState(IntakeControlState.CLOSE);
  }
}
