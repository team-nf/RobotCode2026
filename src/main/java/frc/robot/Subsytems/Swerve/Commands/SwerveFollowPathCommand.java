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
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsytems.Swerve.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveFollowPathCommand extends ParallelCommandGroup {
  /** Creates a new SwerveTeleopCommand. */

  public SwerveFollowPathCommand(CommandSwerveDrivetrain drivetrain, PathPlannerPath path) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    addCommands(
      AutoBuilder.followPath(path),
      new RunCommand(() -> {
        drivetrain.updateSwerveErrors(path.getPathPoses().get(path.getPathPoses().size() - 1));
        drivetrain.setSwerveState(SwerveState.PATH_FOLLOWING);
      })
    );
  }
}
