// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Swerve.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsytems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Utils.States.SwerveStates.SwerveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwervePathFindToPoseCommand extends ParallelCommandGroup {
  /** Creates a new SwerveTeleopCommand. */

  public SwervePathFindToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    addCommands(
      AutoBuilder.pathfindToPose(pose, DriveConstants.PATH_CONSTRAINTS_TO_POSE),
      new RunCommand(() -> {
        drivetrain.updateSwerveErrors(pose);
        drivetrain.setSwerveState(SwerveState.PATH_FOLLOWING);
      })
    );
  }
}
