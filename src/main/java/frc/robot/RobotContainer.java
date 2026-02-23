// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Subsytems.Feeder.FeederSubsystem;
import frc.robot.Subsytems.Hopper.HopperSubsystem;
import frc.robot.Subsytems.Intake.IntakeSubsystem;
import frc.robot.Subsytems.Shooter.ShooterSubsystem;
import frc.robot.Subsytems.Shooter.Utils.ShooterCalculator;
import frc.robot.Subsytems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.Container;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.HopperSim;
import frc.robot.Utils.MatchTracker;
import frc.robot.Utils.ShooterSim;
import frc.robot.Utils.SwerveFieldContactSim;
import frc.robot.Utils.States.IntakeStates.IntakeControlState;
import frc.robot.Utils.States.ShooterStates.ShooterControlState;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private CommandSwerveDrivetrain m_swerveDrivetrain;

  private ShooterCalculator m_shooterCalculator;
  private ShooterSubsystem m_shooterSubsystem;

  private FeederSubsystem m_feederSubsystem;

  
  private HopperSubsystem m_hopperSubsystem;
  
  private IntakeSubsystem m_intakeSubsystem;
  

  private TheMachine m_theMachine;

  private final CommandXboxController m_driverController;

  public RobotContainer() {
    m_feederSubsystem = new FeederSubsystem();

    m_hopperSubsystem = new HopperSubsystem();

    m_intakeSubsystem = new IntakeSubsystem();

    m_swerveDrivetrain = SwerveConstants.createDrivetrain();

    m_shooterCalculator = new ShooterCalculator(m_swerveDrivetrain.swerveDataSupplier());
    m_shooterSubsystem = new ShooterSubsystem(m_shooterCalculator);

    
    m_theMachine = new TheMachine(
                            m_shooterSubsystem, 
                            m_feederSubsystem, 
                            m_hopperSubsystem, 
                            m_intakeSubsystem, 
                            m_swerveDrivetrain.swerveDataSupplier());
    

    //m_ledSubsytem = new LedSubsytem();

    m_driverController = new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

    configureBindings();
  }

  private void configureBindings() {

   //m_swerveDrivetrain.setDefaultCommand(m_swerveDrivetrain.teleopCommand(m_driverController));

    m_driverController.b()
        .onTrue(m_feederSubsystem.feedRequest())
        .onFalse(m_feederSubsystem.zeroRequest());

    m_driverController.a()
        .onTrue(m_hopperSubsystem.feedRequest())
        .onFalse(m_hopperSubsystem.zeroRequest());

    m_driverController.y()
        .onTrue(m_intakeSubsystem.closeRequest());

    m_driverController.x()
        .onTrue(m_intakeSubsystem.intakeRequest())
        .onFalse(m_intakeSubsystem.deployRequest());

    m_driverController.rightBumper()
        .onTrue(m_shooterSubsystem.shootRequest())
        .onFalse(m_shooterSubsystem.restRequest());

     m_driverController.leftBumper()
        .onTrue(m_shooterSubsystem.testRequest())
        .onFalse(m_shooterSubsystem.zeroRequest());

     m_driverController.povDown().onTrue(m_theMachine.shootRequest()).onFalse(m_theMachine.zeroRequest());
  
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {

    if(DriverStation.isEnabled())
    {
      m_theMachine.periodic();
    }

    if (TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE) {
        // Telemetry for the Command Scheduler
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
     }
  }
}
