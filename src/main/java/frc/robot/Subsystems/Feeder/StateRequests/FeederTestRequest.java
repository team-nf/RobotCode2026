// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.States.FeederStates.FeederControlState;
import frc.robot.subsystems.Feeder.FeederSubsystem;

import static edu.wpi.first.units.Units.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederTestRequest extends InstantCommand {

  private final FeederSubsystem feederSubsystem;

  public FeederTestRequest(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setFeederState(FeederControlState.TEST);
  }
}
