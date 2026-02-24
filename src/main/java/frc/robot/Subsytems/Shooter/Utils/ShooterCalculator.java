// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Shooter.Utils;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsytems.Swerve.Utils.SwerveControlData;
import pabeles.concurrency.IntOperatorTask.Min;

/** Add your docs here. */
public class ShooterCalculator {

    Supplier<SwerveControlData> swerveDataSupplier;

    public ShooterCalculator(Supplier<SwerveControlData> swerveDataSupplier) {
        this.swerveDataSupplier = swerveDataSupplier;
    }

    public AngularVelocity calculateFlywheelSpeed(double distanceToTargetMeters, double launchAngleDegrees, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual physics calculations to determine the required wheel speed
        double requiredSpeed = Math.sqrt(9.81 * distanceToTargetMeters); // Simplified example
        return RotationsPerSecond.of(requiredSpeed);
    }

    public Angle calculateHoodAngle(double distanceToTargetMeters, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual calculations to determine the required hood angle
        double angle = Math.toDegrees(Math.atan2(targetHeightMeters - initialHeightMeters, distanceToTargetMeters));
        return Degree.of(angle);
    }

    public AngularVelocity calculateFlywheelSpeedFromCurrentPose()
    {
        Pose2d robotPose = swerveDataSupplier.get().robotPose;
        Transform3d robotPoseTransform = 
                    new Transform3d(new Translation3d(robotPose.getX(), robotPose.getY(), 0), 
                    new Rotation3d(robotPose.getRotation()));

        double hubX = Dimensions.BLUE_HUB_AIM_POSE.getX();
        double hubY = Dimensions.BLUE_HUB_AIM_POSE.getY();
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            hubX = Dimensions.RED_HUB_AIM_POSE.getX();
            hubY = Dimensions.RED_HUB_AIM_POSE.getY();
        }
        Pose3d hubPose =  new Pose3d(hubX, hubY, Dimensions.HUB_HEIGHT.in(Meters), new Rotation3d());


        double hoodAngle = calculateHoodAngleFromCurrentPose().in(Radians);
        double fuelAngle = Math.PI/2 - hoodAngle;

        Pose3d fuelLaunchPose = new Pose3d(
                Dimensions.LEFT_SHOOTER_POSE.getX() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.cos(hoodAngle),
                0.0,
                Dimensions.LEFT_SHOOTER_POSE.getZ() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.sin(hoodAngle),
                new Rotation3d()
            );

        Pose3d wordFuelLaunchPose = fuelLaunchPose.transformBy(robotPoseTransform)
                                                .rotateAround(robotPoseTransform.getTranslation(), new Rotation3d(robotPose.getRotation()));

        double horizontalDistance = Math.sqrt(
            Math.pow(hubPose.getX() - wordFuelLaunchPose.getX(), 2) +
            Math.pow(hubPose.getY() - wordFuelLaunchPose.getY(), 2)
        );

        double verticalDistance = hubPose.getZ() - wordFuelLaunchPose.getZ();

        double t = Math.sqrt((Math.tan(fuelAngle) - verticalDistance / horizontalDistance) * 2 * horizontalDistance / 10);
        
        double requiredVelocity = horizontalDistance / (t * Math.cos(fuelAngle));

        double wheelSpeed = (
            requiredVelocity / (2 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS.in(Meters) * ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT)
        );

        wheelSpeed *= Math.max((hoodAngle/ShooterConstants.MIN_HOOD_ANGLE.in(Radians))/1.3,1);

        wheelSpeed = Math.max(ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond), Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond))); // Clamp between min and max wheel speeds



        return RotationsPerSecond.of(wheelSpeed);
        //return RotationsPerSecond.of(35);
    }


    public Angle calculateHoodAngleFromCurrentPose()
    {
        Pose2d robotPose = swerveDataSupplier.get().robotPose;
        
        double hubX = Dimensions.BLUE_HUB_AIM_POSE.getX();
        double hubY = Dimensions.BLUE_HUB_AIM_POSE.getY();
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            hubX = Dimensions.RED_HUB_AIM_POSE.getX();
            hubY = Dimensions.RED_HUB_AIM_POSE.getY();
        }
        Pose3d hubPose =  new Pose3d(hubX, hubY, Dimensions.HUB_HEIGHT.in(Meters), new Rotation3d());
     
        double distance = Math.sqrt(
            Math.pow(hubPose.getX() - robotPose.getX(), 2) +
            Math.pow(hubPose.getY() - robotPose.getY(), 2)
        );

        double hoodAngle = ShooterConstants.hoodAngleFormula(distance);

        hoodAngle = Math.max(ShooterConstants.MIN_HOOD_ANGLE.in(Degrees), Math.min(hoodAngle, ShooterConstants.MAX_HOOD_ANGLE.in(Degrees))); // Clamp between min and max hood angles

        //System.out.println("Calculated Hood Angle: " + hoodAngle + "Distance: " + distance);
        //return Degrees.of(hoodAngle);
        return Degree.of(hoodAngle);
    }

        public AngularVelocity calculateRestFlywheelSpeedFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate shooter wheel speed based on robot's current pose
        return calculateFlywheelSpeedFromCurrentPose().div(3.0);
    }

    public Angle calculateRestHoodAngleFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate hood angle based on robot's current pose
        return ShooterConstants.MIN_HOOD_ANGLE;
    }
}
