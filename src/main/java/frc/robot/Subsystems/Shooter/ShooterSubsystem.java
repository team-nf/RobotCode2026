// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Subsystems.Shooter.Hardware.ShooterRealHardware;
import frc.robot.Subsystems.Shooter.Hardware.ShooterHardware;
import frc.robot.Subsystems.Shooter.Hardware.ShooterSimHardware;
import frc.robot.Subsystems.Shooter.Utils.ShooterCalculator;
import frc.robot.Subsystems.Shooter.Utils.ShooterControlData;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.States.ShooterStates.*;
import frc.robot.Subsystems.Shooter.StateActions.*;
import frc.robot.Subsystems.Shooter.StateRequests.*;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public final ShooterHardware shooterHardware;
  public final ShooterControlData shooterData;
  public final ShooterCalculator shooterCalculator;

  public final ShooterZeroAction shooterZeroAction;
  public final ShooterRestAction shooterRestAction;
  public final ShooterPrepareAction shooterPrepareAction;
  public final ShooterShootAction shooterShootAction;
  public final ShooterTestAction shooterTestAction;

  public ShooterSubsystem(ShooterCalculator shooterCalculator) 
  {
    if (Utils.isSimulation()) {
        shooterHardware = new ShooterSimHardware();
    } else {
        shooterHardware = new ShooterRealHardware();
    }

    this.shooterCalculator = shooterCalculator;
    shooterData = new ShooterControlData();

    shooterZeroAction = new ShooterZeroAction(this);
    shooterRestAction = new ShooterRestAction(this);
    shooterPrepareAction = new ShooterPrepareAction(this);
    shooterShootAction = new ShooterShootAction(this);
    shooterTestAction = new ShooterTestAction(this);
  }

  public void updateShooterData() {
    shooterData.flywheelVelocity = shooterHardware.getFlywheelVelocity();
    shooterData.hoodAngle = shooterHardware.getHoodPosition();

    shooterData.flywheelError = shooterData.flywheelGoalVelocity.minus(shooterData.flywheelVelocity);
    shooterData.hoodError = shooterData.hoodGoalAngle.minus(shooterData.hoodAngle);

    shooterData.flywheelState = RotationsPerSecond.of(shooterData.flywheelError.abs(RotationsPerSecond))
      .lte(ShooterConstants.FLYWHEEL_ALLOWABLE_ERROR)
        ? FlywheelState.AT_SPEED
        : FlywheelState.REACHING_SPEED;

    shooterData.hoodState = Degrees.of(shooterData.hoodError.abs(Degrees))
      .lte(ShooterConstants.HOOD_ALLOWABLE_ERROR)
      ? HoodState.AT_POSITION
      : HoodState.MOVING_TO_POSITION;
  }

  public void zero() {
    shooterData.flywheelGoalVelocity = RotationsPerSecond.of(0.0);
    shooterData.hoodGoalAngle = Degrees.of(0.0);
    updateShooterData();
    shooterHardware.flywheelStop();
    shooterHardware.hoodZero();
  }

  public void rest() {
    shooterData.flywheelGoalVelocity = shooterCalculator.calculateRestFlywheelSpeedFromCurrentPose();
    shooterData.hoodGoalAngle = shooterCalculator.calculateRestHoodAngleFromCurrentPose();
    updateShooterData();
    shooterHardware.setShooter(
      shooterData.flywheelGoalVelocity, 
      shooterData.hoodGoalAngle);
  }

  public void setShooterFromCurrentPose() {
    shooterData.flywheelGoalVelocity = shooterCalculator.calculateFlywheelSpeedFromCurrentPose();
    shooterData.hoodGoalAngle = shooterCalculator.calculateHoodAngleFromCurrentPose();
    updateShooterData();
    shooterHardware.setShooter(
      shooterData.flywheelGoalVelocity, 
      shooterData.hoodGoalAngle);
  }

  public void test() {
    updateShooterData();
    shooterHardware.testShooter();
  }

  public boolean isShooterState(ShooterControlState state) {
    return shooterData.shooterControlState == state;
  }

  public boolean isPrevShooterState(ShooterControlState state) {
    return shooterData.prevShooterControlState == state;
  }

  public boolean isShooterReadyToShoot() {
    return shooterData.flywheelState == FlywheelState.AT_SPEED
      && shooterData.hoodState == HoodState.AT_POSITION;
  }

  public void setShooterState(ShooterControlState state) {
    if (shooterData.shooterControlState != state) {

        if(state == ShooterControlState.SHOOT_READY && !isShooterState(ShooterControlState.SHOOT_PREPARE)) {
            return; // Prevent skipping prepare state
        }
        shooterData.prevShooterControlState = shooterData.shooterControlState;
        shooterData.shooterControlState = state;
    }
  }

  public InstantCommand zeroRequest() {
    return new ShooterZeroRequest(this);
  }

  public InstantCommand restRequest() {
    return new ShooterRestRequest(this);
  }

  public InstantCommand prepareRequest() {
    return new ShooterPrepareRequest(this);
  }

  public WaitUntilCommand waitForShooterToBeReady() {
    return new WaitUntilCommand(this::isShooterReadyToShoot);
  }

  public InstantCommand testRequest() {
    return new ShooterTestRequest(this);
  }

  public ShooterControlData getShooterData() {
    return shooterData;
  }


  public void stateMachine() {
    switch (shooterData.shooterControlState) {
      case ZERO:
        CommandScheduler.getInstance().schedule(shooterZeroAction);
        break;
      case REST:
        CommandScheduler.getInstance().schedule(shooterRestAction);
        break;
      case SHOOT_PREPARE:
        CommandScheduler.getInstance().schedule(shooterPrepareAction);
        break;
      case SHOOT_READY:
        CommandScheduler.getInstance().schedule(shooterShootAction);
        break;
      case TEST:
        CommandScheduler.getInstance().schedule(shooterTestAction);
        break;
      default:
        CommandScheduler.getInstance().schedule(shooterZeroAction);
        break;
    }
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
        shooterHardware.update();
    }
    
    if (TelemetryConstants.SHOULD_SHOOTER_HARDWARE_COMMUNICATE) {
        SmartDashboard.putData("Shooter Telemetry", shooterHardware);
    }
    if (TelemetryConstants.SHOULD_SHOOTER_CONTROL_COMMUNICATE) {
        SmartDashboard.putData("Shooter Control Data", shooterData);
    }

    stateMachine();
    }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (Utils.isSimulation()) {
        shooterHardware.update();
    }
  }
}
