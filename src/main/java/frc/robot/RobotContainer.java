// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.States.ShooterStates.ShooterControlState;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Hopper.HopperSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.Utils.ShooterCalculator;

public class RobotContainer {

  private final ShooterCalculator m_shooterCalculator;
  private final ShooterSubsystem m_shooterSubsystem;

  private final FeederSubsystem m_feederSubsystem;
  private final HopperSubsystem m_hopperSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  private final CommandXboxController m_driverController;

  public RobotContainer() {
    m_shooterCalculator = new ShooterCalculator();
    m_shooterSubsystem = new ShooterSubsystem(m_shooterCalculator);

    m_feederSubsystem = new FeederSubsystem();

    m_hopperSubsystem = new HopperSubsystem();

    m_intakeSubsystem = new IntakeSubsystem();

    m_driverController = new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a()
        .onTrue(m_shooterSubsystem.prepareRequest()
        .andThen(m_shooterSubsystem.waitForShooterToBeReady()
        .andThen(m_feederSubsystem.feedRequest()
        .alongWith(m_hopperSubsystem.feedRequest()))));

    m_driverController.b()
        .onTrue(m_shooterSubsystem.restRequest());
    m_driverController.x()
        .onTrue(m_shooterSubsystem.testRequest());
    m_driverController.y()
        .onTrue(m_shooterSubsystem.zeroRequest());

    m_driverController.rightBumper()
        .onTrue(m_feederSubsystem.reverseRequest()
            .alongWith(m_hopperSubsystem.reverseRequest()))
        .onFalse(m_feederSubsystem.zeroRequest()
            .alongWith(m_hopperSubsystem.zeroRequest()));

    m_driverController.leftBumper()
        .onTrue(m_intakeSubsystem.intakeRequest())
        .onFalse(m_intakeSubsystem.feedRequest());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
     if (TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE) {
        // Telemetry for the Command Scheduler
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
     }
  }
}
