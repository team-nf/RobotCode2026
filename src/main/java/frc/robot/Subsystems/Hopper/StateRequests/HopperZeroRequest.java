package frc.robot.subsystems.Hopper.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.constants.States.HopperStates.HopperControlState;

public class HopperZeroRequest extends InstantCommand {

  private final HopperSubsystem hopperSubsystem;

  public HopperZeroRequest(HopperSubsystem hopperSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
  }

  @Override
  public void initialize() {
    hopperSubsystem.setHopperState(HopperControlState.ZERO);
  }
}
