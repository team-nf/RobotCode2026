package frc.robot.Subsytems.Intake.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsytems.Intake.IntakeSubsystem;
import frc.robot.Utils.States.IntakeStates.IntakeControlState;

public class IntakeTestRequest extends InstantCommand {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeTestRequest(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeState(IntakeControlState.TEST);
  }
}
