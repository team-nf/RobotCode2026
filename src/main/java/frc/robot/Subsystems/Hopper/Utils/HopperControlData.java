package frc.robot.Subsystems.Hopper.Utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Constants.States.HopperStates.HopperRollerState;

public class HopperControlData implements Sendable {

    public HopperRollerState hopperRollerState = HopperRollerState.ZERO;
    public HopperControlState hopperControlState = HopperControlState.ZERO;
    public HopperControlState prevHopperControlState = HopperControlState.ZERO;

    public AngularVelocity hopperGoalVelocity = RotationsPerSecond.of(0);
    public AngularVelocity hopperVelocity = RotationsPerSecond.of(0);
    public AngularVelocity hopperError = RotationsPerSecond.of(0);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HopperControlData");
        builder.addStringProperty("HopperControlState", () -> hopperControlState.toString(), null);
        builder.addStringProperty("HopperPrevControlState", () -> prevHopperControlState.toString(), null);
        builder.addStringProperty("HopperRollerState", () -> hopperRollerState.toString(), null);
        builder.addDoubleProperty("HopperGoalRPM", () -> hopperGoalVelocity.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("HopperRPM", () -> hopperVelocity.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("HopperErrorRPM", () -> hopperError.in(RotationsPerSecond)*60, null);
    }
}
