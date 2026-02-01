// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Utils.ShooterSim;
import frc.robot.Utils.States.IntakeStates.IntakeControlState;
import frc.robot.Utils.States.ShooterStates.ShooterControlState;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private final CommandSwerveDrivetrain m_swerveDrivetrain;

  private final ShooterCalculator m_shooterCalculator;
  private final ShooterSubsystem m_shooterSubsystem;

  private final FeederSubsystem m_feederSubsystem;
  private final HopperSubsystem m_hopperSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;


  private final TheMachine m_theMachine;

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

    m_driverController = new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

    configureBindings();
    if(Utils.isSimulation()) configureFuelSim();
  }

  private void configureBindings() {

   m_swerveDrivetrain.setDefaultCommand(m_swerveDrivetrain.teleopCommand(m_driverController));

    m_driverController.b()
        .onTrue(m_theMachine.reverseRequest());

    m_driverController.y()
        .whileTrue(m_swerveDrivetrain.aimToHub().andThen(m_theMachine.shootRequest()))
        .onFalse(m_theMachine.idleDeployedRequest());

    m_driverController.a()
        .onTrue(m_theMachine.idleRetractedRequest());

    m_driverController.x()
        .onTrue(m_theMachine.intakeRequest());

  }

  private void configureFuelSim() {
    FuelSim fuelSim = FuelSim.getInstance();
    HopperSim hopperSim = HopperSim.getInstance();
    ShooterSim shooterSim = ShooterSim.getInstance();

    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(
            Dimensions.BUMPER_WIDTH.in(Meters),
            Dimensions.BUMPER_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            m_swerveDrivetrain::getPose,
            m_swerveDrivetrain::getFieldSpeeds);
    
    fuelSim.registerIntake(
            Dimensions.BUMPER_LENGTH.div(2).in(Meters),
            Dimensions.BUMPER_LENGTH.div(2).plus(Dimensions.HOPPER_EXTENSION_LENGTH).in(Meters),
            -Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            () -> m_intakeSubsystem.isIntakeDeployed() 
                        && m_intakeSubsystem.isIntakeState(IntakeControlState.INTAKE) 
                        && hopperSim.isHopperAbleToIntake(),
            hopperSim::addFuelToHopper);

    fuelSim.start();

    hopperSim.setRobotPoseSupplier(m_swerveDrivetrain::getPose);
    hopperSim.setShouldRemoveFuelSupplier(() -> m_intakeSubsystem.isIntakeDeployed() 
                        && m_intakeSubsystem.isIntakeState(IntakeControlState.REVERSE));

    shooterSim.setShooterControlDataSupplier(m_shooterSubsystem::getCurrentControlData);
    shooterSim.setRobotPoseSupplier(m_swerveDrivetrain::getPose);
    shooterSim.setChassisSpeedsSupplier(m_swerveDrivetrain::getFieldSpeeds);
    shooterSim.setShouldShootSupplier(() -> m_shooterSubsystem.isShooterState(ShooterControlState.SHOOT));

    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
}

private Pose2d currentSimPose = Container.START_POSE_BLUE;
private Pose2d prevSimPose = Container.START_POSE_BLUE;

public void handleSwerveSimFieldCollisions() {
        currentSimPose = m_swerveDrivetrain.getState().Pose;

        // Handle collisions with field boundaries
        double fieldXMin = 0.1;
        double fieldXMax = 16.33;
        double fieldYMin = 0.1;
        double fieldYMax = 8.15;

        boolean x_collided = false;
        boolean y_collided = false;

        double frontCollisionDistance = Dimensions.BUMPER_COLLISION_DISTANCE.in(Meters) / 2.0;
        double backCollisionDistance = Dimensions.BUMPER_COLLISION_DISTANCE.in(Meters) / 2.0;

        if(m_intakeSubsystem.isIntakeDeployed()) {
            frontCollisionDistance += Dimensions.HOPPER_EXTENSION_LENGTH.in(Meters);
        }
        
        Pose2d frontLeftCorner = currentSimPose.transformBy(
            new Transform2d(
                frontCollisionDistance * Math.cos(currentSimPose.getRotation().getRadians() + Math.PI/4),
                frontCollisionDistance * Math.sin(currentSimPose.getRotation().getRadians() + Math.PI/4),
                Rotation2d.kZero
            )
        );

        Pose2d frontRightCorner = currentSimPose.transformBy(
            new Transform2d(
                frontCollisionDistance * Math.cos(currentSimPose.getRotation().getRadians() - Math.PI/4),
                frontCollisionDistance * Math.sin(currentSimPose.getRotation().getRadians() - Math.PI/4),
                Rotation2d.kZero
            )
        );

        Pose2d backLeftCorner = currentSimPose.transformBy(
            new Transform2d(
                backCollisionDistance * Math.cos(currentSimPose.getRotation().getRadians() + 3*Math.PI/4),
                backCollisionDistance * Math.sin(currentSimPose.getRotation().getRadians() + 3*Math.PI/4),
                Rotation2d.kZero
            )
        );

        Pose2d backRightCorner = currentSimPose.transformBy(
            new Transform2d(
                backCollisionDistance * Math.cos(currentSimPose.getRotation().getRadians() - 3*Math.PI/4),
                backCollisionDistance * Math.sin(currentSimPose.getRotation().getRadians() - 3*Math.PI/4),
                Rotation2d.kZero
            )
        );

        boolean[] collisionResults = Dimensions.checkFieldCollision(new Pose2d[] {
            frontLeftCorner,
            frontRightCorner,
            backLeftCorner,
            backRightCorner
        });

        x_collided = collisionResults[0];
        y_collided = collisionResults[1];

        double correctedX = currentSimPose.getX();
        double correctedY = currentSimPose.getY();

        if (x_collided) correctedX = prevSimPose.getX();
        if (y_collided) correctedY = prevSimPose.getY();

        if (x_collided || y_collided) {
            m_swerveDrivetrain.resetPose(new Pose2d(correctedX, correctedY, prevSimPose.getRotation()));
        }
        else prevSimPose = currentSimPose;
        
    }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {

    m_theMachine.periodic();

    if (TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE) {
        // Telemetry for the Command Scheduler
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
     }
  }
}
