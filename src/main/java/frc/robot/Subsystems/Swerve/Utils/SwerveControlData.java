package frc.robot.subsystems.Swerve.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.States.SwerveStates.SwerveState;

public class SwerveControlData implements Sendable {

    public SwerveState swerveControlState = SwerveState.TELEOP;

    public Field2d field = new Field2d();
    public Pose2d robotPose = new Pose2d();

    public double robotVelocityX = 0.0;
    public double robotVelocityY = 0.0;
    public double robotSpeed = 0.0;
    public double robotAngularVelocity = 0.0;   

    @Override
    public void initSendable(SendableBuilder builder) {

        field.setRobotPose(robotPose);
        field.initSendable(builder);

        builder.setSmartDashboardType("SwerveControlData");
        builder.addStringProperty("SwerveControlState", () -> swerveControlState.toString(), null);
        builder.addDoubleProperty("RobotVelocityX", () -> robotVelocityX, null);
        builder.addDoubleProperty("RobotVelocityY", () -> robotVelocityY, null);
        builder.addDoubleProperty("RobotSpeed", () -> robotSpeed, null);
        builder.addDoubleProperty("RobotAngularVelocity", () -> robotAngularVelocity, null);
    }

}