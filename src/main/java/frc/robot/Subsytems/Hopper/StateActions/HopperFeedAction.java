package frc.robot.Subsytems.Hopper.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Hopper.HopperSubsystem;
import frc.robot.Utils.States.HopperStates.HopperControlState;

public class HopperFeedAction extends Command {

  private final HopperSubsystem hopperSubsystem;

  public HopperFeedAction(HopperSubsystem hopperSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void initialize() {
    hopperSubsystem.feed();
  }

  @Override
  public void execute() {
    hopperSubsystem.feed();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !hopperSubsystem.isHopperState(HopperControlState.FEED);
  }
}
