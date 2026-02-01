package frc.robot.Subsytems.Hopper.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Hopper.HopperSubsystem;
import frc.robot.Utils.States.HopperStates.HopperControlState;

public class HopperPushAction extends Command {

    private final HopperSubsystem hopperSubsystem;

    public HopperPushAction(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization logic for pushing
    }

    @Override
    public void execute() {
        hopperSubsystem.push(); // Call the push method in HopperSubsystem
    }

    @Override
    public boolean isFinished() {
        return !hopperSubsystem.isHopperState(HopperControlState.PUSH); // Modify this condition as needed
    }
}