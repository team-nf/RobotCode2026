package frc.robot.Subsytems.Swerve.Utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utils.States.SwerveStates.SwerveState;

public class SwerveControlData implements Sendable {

    public SwerveState swerveControlState = SwerveState.TELEOP;

    public Field2d field = new Field2d();
    public Pose2d robotPose = new Pose2d();

    public LinearVelocity robotVelocityX = MetersPerSecond.of(0.0);
    public LinearVelocity robotVelocityY = MetersPerSecond.of(0.0);
    public LinearVelocity robotSpeed = MetersPerSecond.of(0.0);
    public AngularVelocity robotAngularVelocity = RotationsPerSecond.of(0.0);

    public boolean isAimed = false;

    public Distance robotXError = Meters.of(0.0);
    public Distance robotYError = Meters.of(0.0);
    public Angle robotAngleError = Degrees.of(0.0);

    public double fuelStorage = 0.0;

    public PIDController aimingPID = new PIDController(
        DriveConstants.AIMING_kP,
        DriveConstants.AIMING_kI,
        DriveConstants.AIMING_kD
    );

    @Override
    public void initSendable(SendableBuilder builder) {

        field.setRobotPose(robotPose);

        builder.setSmartDashboardType("SwerveControlData");

        builder.addStringProperty("SwerveControlState", () -> swerveControlState.toString(), null);

        builder.addStringProperty("RobotPoseString", () -> robotPose.toString(), null);

        builder.addDoubleProperty("RobotVelocityX", () -> robotVelocityX.in(MetersPerSecond), null);
        builder.addDoubleProperty("RobotVelocityY", () -> robotVelocityY.in(MetersPerSecond), null);
        builder.addDoubleProperty("RobotSpeed", () -> robotSpeed.in(MetersPerSecond), null);
        builder.addDoubleProperty("RobotAngularVelocity", () -> robotAngularVelocity.in(RotationsPerSecond), null);

        builder.addDoubleProperty("RobotXError", () -> robotXError.in(Meters), null);
        builder.addDoubleProperty("RobotYError", () -> robotYError.in(Meters), null);
        builder.addDoubleProperty("RobotAngleError", () -> robotAngleError.in(Degrees), null);

        builder.addBooleanProperty("IsAimed", () -> isAimed, null);

        builder.addDoubleProperty("AimingPID_P", () -> aimingPID.getP(), (value) -> aimingPID.setP(value));
        builder.addDoubleProperty("AimingPID_I", () -> aimingPID.getI(), (value) -> aimingPID.setI(value));
        builder.addDoubleProperty("AimingPID_D", () -> aimingPID.getD(), (value) -> aimingPID.setD(value));
    }

}