// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.States.ShooterStates.FlywheelState;
import frc.robot.Constants.States.ShooterStates.HoodState;
import frc.robot.Constants.States.ShooterStates.ShooterControlState;
import frc.robot.Subsytems.Shooter.Hardware.ShooterHardware;
import frc.robot.Subsytems.Shooter.Hardware.ShooterRealHardware;
import frc.robot.Subsytems.Shooter.Hardware.ShooterSimHardware;
import frc.robot.Subsytems.Shooter.StateActions.ShooterRestAction;
import frc.robot.Subsytems.Shooter.StateActions.ShooterShootAction;
import frc.robot.Subsytems.Shooter.StateActions.ShooterTestAction;
import frc.robot.Subsytems.Shooter.StateActions.ShooterZeroAction;
import frc.robot.Subsytems.Shooter.StateRequests.*;
import frc.robot.Subsytems.Shooter.Utils.ShooterCalculator;
import frc.robot.Subsytems.Shooter.Utils.ShooterControlData;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public final ShooterHardware shooterHardware;
  public final ShooterControlData shooterData;
  public final ShooterCalculator shooterCalculator;

  public final Command shooterZeroAction;
  public final Command shooterRestAction;
  public final Command shooterShootAction;
  public final Command shooterTestAction;

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
    shooterShootAction = new ShooterShootAction(this);
    shooterTestAction = new ShooterTestAction(this);
  }

  public void updateShooterData() {
    shooterData.flywheelVelocityL = shooterHardware.getFlywheelVelocityL();
    shooterData.flywheelVelocityR = shooterHardware.getFlywheelVelocityR();
    shooterData.hoodAngle = shooterHardware.getHoodPosition();

    shooterData.flywheelErrorL = shooterData.flywheelGoalVelocity.minus(shooterData.flywheelVelocityL);
    shooterData.flywheelErrorR = shooterData.flywheelGoalVelocity.minus(shooterData.flywheelVelocityR);
    shooterData.hoodError = shooterData.hoodGoalAngle.minus(shooterData.hoodAngle);

    shooterData.flywheelStateL = RotationsPerSecond.of(shooterData.flywheelErrorL.abs(RotationsPerSecond))
      .lte(ShooterConstants.FLYWHEEL_ALLOWABLE_ERROR)
        ? FlywheelState.AT_SPEED
        : FlywheelState.REACHING_SPEED;

    shooterData.flywheelStateR = RotationsPerSecond.of(shooterData.flywheelErrorR.abs(RotationsPerSecond))
      .lte(ShooterConstants.FLYWHEEL_ALLOWABLE_ERROR)
        ? FlywheelState.AT_SPEED
        : FlywheelState.REACHING_SPEED;

    shooterData.hoodState = Degrees.of(shooterData.hoodError.abs(Degrees))
      .lte(ShooterConstants.HOOD_ALLOWABLE_ERROR.times(ShooterConstants.HOOD_GEAR_REDUCTION))
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
    shooterData.flywheelGoalVelocity = shooterHardware.getTestFlywheelGoal();
    shooterData.hoodGoalAngle = shooterHardware.getTestHoodGoal();
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
    return shooterData.flywheelStateL == FlywheelState.AT_SPEED
      && shooterData.flywheelStateR == FlywheelState.AT_SPEED
      && shooterData.hoodState == HoodState.AT_POSITION;
  }

  public void setShooterState(ShooterControlState state) {
    if (shooterData.shooterControlState != state) {

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

  public InstantCommand shootRequest() {
    return new ShooterShootRequest(this);
  }

  public InstantCommand testRequest() {
    return new ShooterTestRequest(this);
  }

  public WaitUntilCommand waitForShooterToBeReady() {
    return new WaitUntilCommand(this::isShooterReadyToShoot);
  }



  public ShooterControlData getShooterData() {
    return shooterData;
  }

  public Angle getHoodAngleInRealLife() {
    return shooterHardware.getHoodPosition();
  }

  public ShooterControlData getCurrentControlData() {
    return shooterData;
  }

  public void stateMachine() {
    switch (shooterData.shooterControlState) {
      case ZERO:
        if(!CommandScheduler.getInstance().isScheduled(shooterZeroAction)) {
          CommandScheduler.getInstance().schedule(shooterZeroAction);
        }
        break;
      case REST:
        if(!CommandScheduler.getInstance().isScheduled(shooterRestAction)) {
          CommandScheduler.getInstance().schedule(shooterRestAction);
        }
        break;
      case SHOOT:
        if(!CommandScheduler.getInstance().isScheduled(shooterShootAction)) {
          CommandScheduler.getInstance().schedule(shooterShootAction);
        }
        break;
      case TEST:
        if(!CommandScheduler.getInstance().isScheduled(shooterTestAction)) {
          CommandScheduler.getInstance().schedule(shooterTestAction);
        }
        break;
      default:
        if(!CommandScheduler.getInstance().isScheduled(shooterZeroAction)) {
          CommandScheduler.getInstance().schedule(shooterZeroAction);
        }
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
