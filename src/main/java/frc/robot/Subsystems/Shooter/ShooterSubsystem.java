// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelemetryConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final ShooterHardware shooterHardware;

  public ShooterSubsystem() 
  {
    if (Utils.isSimulation()) {
        shooterHardware = new SimShooterHardware();
    } else {
        shooterHardware = new RealShooterHardware();
    }

  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
        shooterHardware.update();
    }
    
    if (TelemetryConstants.shouldShooterCommunicate) {
        SmartDashboard.putData("Shooter Telemetry", shooterHardware);
    }

    shooterHardware.testShooter();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (Utils.isSimulation()) {
        shooterHardware.update();
    }
  }
}
