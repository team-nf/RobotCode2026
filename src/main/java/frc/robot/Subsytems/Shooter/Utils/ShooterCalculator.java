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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsytems.Swerve.Utils.SwerveControlData;
import pabeles.concurrency.IntOperatorTask.Min;

/** Add your docs here. */
public class ShooterCalculator {

    Supplier<SwerveControlData> swerveDataSupplier;

    public ShooterCalculator(Supplier<SwerveControlData> swerveDataSupplier) {
        this.swerveDataSupplier = swerveDataSupplier;
    }

    public double calculateFlywheelSpeed(double distanceToTargetMeters, double launchAngleDegrees, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual physics calculations to determine the required wheel speed
        double requiredSpeed = Math.sqrt(9.81 * distanceToTargetMeters); // Simplified example
        return requiredSpeed;
    }

    public double calculateHoodAngle(double distanceToTargetMeters, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual calculations to determine the required hood angle
        double angle = Math.toDegrees(Math.atan2(targetHeightMeters - initialHeightMeters, distanceToTargetMeters));
        return angle/360;
    }


    public double calculateFlywheelSpeedFromCurrentPose()
    {

        double wheelSpeed = ShooterConstants.flywheelRPMFormula(swerveDataSupplier.get().distanceToHub.in(Meters));

        wheelSpeed /= 60;
        wheelSpeed /= ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;

        SmartDashboard.putNumber("WheelSpeedCalculated", wheelSpeed);

        wheelSpeed = Math.max(ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond), Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond))); // Clamp between min and max wheel speeds

        return wheelSpeed;
        //return RotationsPerSecond.of(35);
    }


    public double calculateHoodAngleFromCurrentPose()
    {
        double hoodAngle = ShooterConstants.hoodAngleFormula(swerveDataSupplier.get().distanceToHub.in(Meters));

        hoodAngle = Math.max(ShooterConstants.MIN_HOOD_ANGLE.in(Degrees), Math.min(hoodAngle, ShooterConstants.MAX_HOOD_ANGLE.in(Degrees))); // Clamp between min and max hood angles

        //System.out.println("Calculated Hood Angle: " + hoodAngle + "Distance: " + distance);
        //return Degrees.of(hoodAngle);
        return hoodAngle/360;
    }

        public double calculateRestFlywheelSpeedFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate shooter wheel speed based on robot's current pose
        return ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);//.div((3.0))
    }

    public double calculateRestHoodAngleFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate hood angle based on robot's current pose
        return ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);
    }
}
