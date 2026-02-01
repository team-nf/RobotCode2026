package frc.robot.Subsytems.Hopper.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsytems.Hopper.HopperSubsystem;
import frc.robot.Utils.States.HopperStates.HopperControlState;

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
