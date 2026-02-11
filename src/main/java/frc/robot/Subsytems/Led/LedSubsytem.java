// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Led;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Dimensions;
import frc.robot.Utils.MatchTracker;
import frc.robot.Utils.SwerveFieldContactSim;

public class LedSubsytem extends SubsystemBase {
  /** Creates a new LedSubsytem. */
  private Pose3d[] ledPoses = new Pose3d[4];
  private Transform3d[] ledPosesRelative = new Transform3d[4];
  private Pose3d[] offPoses = new Pose3d[4];

  private boolean isBlueHubActive = false;
  private boolean isRedHubActive = false;

  public LedSubsytem() {

    double ledDistance = Dimensions.BUMPER_WIDTH.div(2).in(Meter);

    ledPosesRelative[0] = new Transform3d(new Translation3d(ledDistance, ledDistance, 0.2), new Rotation3d(0,0,0));
    ledPosesRelative[1] = new Transform3d(new Translation3d(-ledDistance, ledDistance, 0.2), new Rotation3d(0,0,0));
    ledPosesRelative[2] = new Transform3d(new Translation3d(ledDistance, -ledDistance, 0.2), new Rotation3d(0,0,0));
    ledPosesRelative[3] = new Transform3d(new Translation3d(-ledDistance, -ledDistance, 0.2), new Rotation3d(0,0,0));

    offPoses[0] = new Pose3d(0, 0, 20, new Rotation3d());
    offPoses[1] = new Pose3d(0, 0, 20, new Rotation3d());
    offPoses[2] = new Pose3d(0, 0, 20, new Rotation3d());
    offPoses[3] = new Pose3d(0, 0, 20, new Rotation3d());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  private StructArrayPublisher<Pose3d> blueLedPublisher 
    = NetworkTableInstance.getDefault().getStructArrayTopic("Led/BlueLedPoses", Pose3d.struct).publish();

  private StructArrayPublisher<Pose3d> redLedPublisher 
    = NetworkTableInstance.getDefault().getStructArrayTopic("Led/RedLedPoses", Pose3d.struct).publish();

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    Pose2d robotPose2d = SwerveFieldContactSim.getInstance().getPose();

    Pose3d robotPose3d = new Pose3d(new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0), new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

    for (int i = 0; i < ledPoses.length; i++) {
      ledPoses[i] =  robotPose3d.transformBy(ledPosesRelative[i]);
    }

    isBlueHubActive = MatchTracker.getInstance().isBlueHubActive();
    isRedHubActive = MatchTracker.getInstance().isRedHubActive();

    if (isBlueHubActive) {
      blueLedPublisher.set(ledPoses);
      redLedPublisher.set(offPoses);
    } else if(isRedHubActive) {
      redLedPublisher.set(ledPoses);
      blueLedPublisher.set(offPoses);
    }
    else {
      blueLedPublisher.set(offPoses);
      redLedPublisher.set(offPoses);
    }
  }
}
