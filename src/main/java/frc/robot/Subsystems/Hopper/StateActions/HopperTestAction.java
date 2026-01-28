package frc.robot.subsystems.Hopper.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.constants.States.HopperStates.HopperControlState;

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
    hopperSubsystem.test();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !hopperSubsystem.isHopperState(HopperControlState.TEST);
  }
}
