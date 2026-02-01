package frc.robot.Subsytems.Shooter.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Utils.States.ShooterStates.*;

public class ShooterControlData implements Sendable{

    public FlywheelState flywheelState = FlywheelState.ZERO;
    public HoodState hoodState = HoodState.ZERO;
    public ShooterControlState shooterControlState = ShooterControlState.ZERO;
    public ShooterControlState prevShooterControlState = ShooterControlState.ZERO;

    public AngularVelocity flywheelGoalVelocity = RotationsPerSecond.of(0.0);
    public Angle hoodGoalAngle = Degrees.of(0.0);

    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0);
    public Angle hoodAngle = Degrees.of(0.0);

    public AngularVelocity flywheelError = RotationsPerSecond.of(0.0);
    public Angle hoodError = Degrees.of(0.0);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterControlData");
        builder.addStringProperty("ShooterControlState", () -> shooterControlState.toString(), null);
        builder.addStringProperty("PrevShooterControlState", () -> prevShooterControlState.toString(), null);
        builder.addStringProperty("FlywheelState", () -> flywheelState.toString(), null);
        builder.addStringProperty("HoodState", () -> hoodState.toString(), null);
        builder.addDoubleProperty("FlywheelGoalRPM", () -> flywheelGoalVelocity.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("HoodGoalAngleDegrees", () -> hoodGoalAngle.in(Degrees), null);
        builder.addDoubleProperty("FlywheelRPM", () -> flywheelVelocity.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("HoodAngleDegrees", () -> hoodAngle.in(Degrees), null);
        builder.addDoubleProperty("FlywheelErrorRPM", () -> flywheelError.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("HoodErrorDegrees", () -> hoodError.in(Degrees), null);
    }

}
