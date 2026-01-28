// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Utils;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/** Add your docs here. */
public class ShooterCalculator {
    public ShooterCalculator() {
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
        // Placeholder implementation
        // Replace with actual logic to calculate shooter wheel speed based on robot's current pose
        return RotationsPerSecond.of(1500/60);
    }

    public Angle calculateHoodAngleFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate hood angle based on robot's current pose
        return Degrees.of(10);
    }

        public AngularVelocity calculateRestFlywheelSpeedFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate shooter wheel speed based on robot's current pose
        return RotationsPerSecond.of(1000/60);
    }

    public Angle calculateRestHoodAngleFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate hood angle based on robot's current pose
        return Degrees.of(5);
    }
}
