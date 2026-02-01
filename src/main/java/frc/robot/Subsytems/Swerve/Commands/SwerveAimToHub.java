// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Swerve.Commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsytems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Utils.States.SwerveStates.SwerveState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveAimToHub extends Command {
  /** Creates a new SwerveAimToHub. */

  private final CommandSwerveDrivetrain swerveDrivetrain;

  private PIDController aimingPID;

  private Pose2d hubAimPose;

  private double MaxSpeed = 0.75 * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public SwerveAimToHub(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;

    aimingPID = swerveDrivetrain.getAimingPID();
    addRequirements(swerveDrivetrain);

    if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue))
    {
      hubAimPose = new Pose2d(4.61, 4.1, new Rotation2d());
    }
    else
    {
      hubAimPose = new Pose2d(11.92, 4.1, new Rotation2d());
    }

    hubAimPose = new Pose2d(4.61, 4.1, new Rotation2d());

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimingPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerveDrivetrain.getPose();
    double robotAngleToHub = Math.atan2(hubAimPose.getY() - robotPose.getY(), hubAimPose.getX() - robotPose.getX());

    double angleError = robotAngleToHub - robotPose.getRotation().getRadians();

    // Normalize angle error to the range [-pi, pi]
    angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
    double output = aimingPID.calculate(-angleError);

    swerveDrivetrain.updateSwerveErrors(robotPose.plus(new Transform2d(0.0, 0.0, new Rotation2d(angleError))));
    swerveDrivetrain.setSwerveState(SwerveState.AIMING);
    swerveDrivetrain.updateSwerveData();
    
    swerveDrivetrain.setControl(drive
      .withRotationalRate(output)
    );

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(aimingPID.getError()) < DriveConstants.AIMING_TOLERANCE_RADIANS;
  }
}
