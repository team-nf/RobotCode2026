package frc.robot.Subsytems.Shooter.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.States.ShooterStates.*;
import frc.robot.Constants.TelemetryConstants;

public class ShooterControlData implements Sendable{

    public FlywheelState flywheelStateL = FlywheelState.ZERO;
    public FlywheelState flywheelStateR = FlywheelState.ZERO;

    public HoodState hoodState = HoodState.ZERO;
    public ShooterControlState shooterControlState = ShooterControlState.ZERO;
    public ShooterControlState prevShooterControlState = ShooterControlState.ZERO;

    public AngularVelocity flywheelGoalVelocity = RotationsPerSecond.of(0.0);
    public Angle hoodGoalAngle = Degrees.of(0.0);

    public AngularVelocity flywheelVelocityL = RotationsPerSecond.of(0.0);
    public AngularVelocity flywheelVelocityR = RotationsPerSecond.of(0.0);
    public Angle hoodAngle = Degrees.of(0.0);

    public AngularVelocity flywheelErrorL = RotationsPerSecond.of(0.0);
    public AngularVelocity flywheelErrorR = RotationsPerSecond.of(0.0);
    public Angle hoodError = Degrees.of(0.0);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterControlData");
        builder.addStringProperty("ShooterControlState", () -> shooterControlState.toString(), null);
        builder.addStringProperty("PrevShooterControlState", () -> prevShooterControlState.toString(), null);
        builder.addStringProperty("FlywheelState-L", () -> flywheelStateL.toString(), null);
        builder.addStringProperty("FlywheelState-R", () -> flywheelStateR.toString(), null);
        builder.addStringProperty("HoodState", () -> hoodState.toString(), null);
        builder.addDoubleProperty("FlywheelGoalRPM", () -> TelemetryConstants.roundTelemetry(flywheelGoalVelocity.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("HoodGoalAngleDegrees", () -> TelemetryConstants.roundTelemetry(hoodGoalAngle.in(Degrees)), null);
        builder.addDoubleProperty("FlywheelRPM-L", () -> TelemetryConstants.roundTelemetry(flywheelVelocityL.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("FlywheelRPM-R", () -> TelemetryConstants.roundTelemetry(flywheelVelocityR.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("HoodAngleDegrees", () -> TelemetryConstants.roundTelemetry(hoodAngle.in(Degrees)), null);
        builder.addDoubleProperty("FlywheelErrorRPM-L", () -> TelemetryConstants.roundTelemetry(flywheelErrorL.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("FlywheelErrorRPM-R", () -> TelemetryConstants.roundTelemetry(flywheelErrorR.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("HoodErrorDegrees", () -> TelemetryConstants.roundTelemetry(hoodError.in(Degrees)), null);
    }

}
