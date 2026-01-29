package frc.robot.Subsytems.Intake.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Subsytems.Intake.IntakeSubsystem;

public class IntakeReverseRequest extends InstantCommand {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeReverseRequest(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeState(IntakeControlState.REVERSE);
  }
}
