package frc.robot.Subsytems.Hopper.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Subsytems.Hopper.HopperSubsystem;

public class HopperTestAction extends Command {

  private final HopperSubsystem hopperSubsystem;

  public HopperTestAction(HopperSubsystem hopperSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void initialize() {
    hopperSubsystem.test();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !hopperSubsystem.isHopperState(HopperControlState.TEST);
  }
}
