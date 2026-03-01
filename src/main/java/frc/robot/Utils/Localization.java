package frc.robot.Utils;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsytems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.PoseEstimate;


public class Localization {
    CommandSwerveDrivetrain drivetrain;
    LimelightHelpers.PoseEstimate currentPoseEstimateFinal;
    boolean useMT2;

    private boolean isMode1Set = false;
    private boolean isLLReady = false;

    public Localization(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.useMT2 = true;

        // Switch to internal IMU with external assist when enabled
        //LimelightHelpers.SetIMUAssistAlpha("limelight-left", 0.001);  // Adjust correction strength

        // Switch to internal IMU with external assist when enabled
        //LimelightHelpers.SetIMUAssistAlpha("limelight-right", 0.001);  // Adjust correction strength

        SmartDashboard.putBoolean("LL-Left_Enabled", true);
        SmartDashboard.putBoolean("LL-Right_Enabled", true);

    }

    public void addVisionMeasurement1() {
        //LimelightHelpers.setPipelineIndex("", 0); // sonra sil
        //if (LimelightHelpers.getCurrentPipelineIndex("") == 0) {
            double robotYaw = drivetrain.getGyroHeading();
            //sSystem.out.println(robotYaw);
            LimelightHelpers.SetIMUMode("limelight-left",2);
            LimelightHelpers.SetIMUMode("limelight-right",2);
            LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
            LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
            LimelightHelpers.PoseEstimate ll_measurement_left;
            LimelightHelpers.PoseEstimate ll_measurement_right;
            
            if (useMT2) {
                ll_measurement_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
                ll_measurement_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
            } else {
                ll_measurement_left = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
                ll_measurement_right = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
            }
           
            LimelightHelpers.PoseEstimate finalEstimate = null;

            // Initial visibility check based on tag count
            boolean is_ll_left_seeing = (ll_measurement_left != null && ll_measurement_left.tagCount > 0);
            boolean is_ll_right_seeing = (ll_measurement_right != null && ll_measurement_right.tagCount > 0);

            // Drop high-ambiguity measurements
            if (is_ll_left_seeing && ll_measurement_left.rawFiducials != null && ll_measurement_left.rawFiducials.length > 0) {
                if (ll_measurement_left.rawFiducials[0].ambiguity > 0.7) {
                    ll_measurement_left = null;
                }
            }
            if (is_ll_right_seeing && ll_measurement_right.rawFiducials != null && ll_measurement_right.rawFiducials.length > 0) {
                if (ll_measurement_right.rawFiducials[0].ambiguity > 0.7) {
                    ll_measurement_right = null;
                }
            }

            // Recompute visibility after possibly nulling out measurements
            is_ll_left_seeing = (ll_measurement_left != null && ll_measurement_left.tagCount > 0);
            is_ll_right_seeing = (ll_measurement_right != null && ll_measurement_right.tagCount > 0);

            if (is_ll_left_seeing && is_ll_right_seeing) {
                boolean leftHasFid = ll_measurement_left.rawFiducials != null && ll_measurement_left.rawFiducials.length > 0;
                boolean rightHasFid = ll_measurement_right.rawFiducials != null && ll_measurement_right.rawFiducials.length > 0;

                if (leftHasFid && rightHasFid
                        && ll_measurement_left.rawFiducials[0].ambiguity < 0.4
                        && ll_measurement_right.rawFiducials[0].ambiguity < 0.4) {
                    double avgx = (ll_measurement_left.pose.getX() + ll_measurement_right.pose.getX()) / 2.0;
                    double avgy = (ll_measurement_left.pose.getY() + ll_measurement_right.pose.getY()) / 2.0;
                    double avgHeading = (ll_measurement_left.pose.getRotation().getDegrees()
                            + ll_measurement_right.pose.getRotation().getDegrees()) / 2.0;
                    Pose2d avgPose2d = new Pose2d(new Translation2d(avgx, avgy), Rotation2d.fromDegrees(avgHeading));
                    finalEstimate = new LimelightHelpers.PoseEstimate(
                            avgPose2d,
                            ll_measurement_left.timestampSeconds,
                            ll_measurement_left.latency,
                            ll_measurement_left.tagCount,
                            ll_measurement_left.tagSpan,
                            ll_measurement_left.avgTagDist,
                            ll_measurement_left.avgTagArea,
                            ll_measurement_left.rawFiducials,
                            ll_measurement_left.isMegaTag2);
                } else if (leftHasFid && rightHasFid
                        && ll_measurement_left.rawFiducials[0].ambiguity > ll_measurement_right.rawFiducials[0].ambiguity) {
                    finalEstimate = ll_measurement_right;
                } else {
                    finalEstimate = ll_measurement_left;
                }
            } else if (is_ll_left_seeing) {
                finalEstimate = ll_measurement_left;
            } else if (is_ll_right_seeing) {
                finalEstimate = ll_measurement_right;
            }

            if ((finalEstimate != null)&&(finalEstimate.tagCount > 0)) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                drivetrain.addVisionMeasurement(
                        finalEstimate.pose,
                        finalEstimate.timestampSeconds);
                currentPoseEstimateFinal = finalEstimate;
                //SmartDashboard.putNumber("finalx", finalEstimate.pose.getX());
            }
    }

    public void addVisionMeasurement() {
        // First, tell Limelight your robot's current orientation
    double robotYaw = drivetrain.getGyroHeading();
    LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurementLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

    // Add it to your pose estimator
    drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

    
    boolean doRejectUpdate = false;
    boolean doRejectLeft = false;
    boolean doRejectRight = false;
    
    if(Math.abs(drivetrain.getGyroRate()) > 360)
    {
        doRejectUpdate = true;
    }

    if(limelightMeasurementLeft != null)
    {
    if(limelightMeasurementLeft.tagCount == 0)
        {
            doRejectLeft = true;
        }
    }
    else doRejectLeft = true;

    if(limelightMeasurementRight != null)
    {
    if(limelightMeasurementRight.tagCount == 0)
        {
            doRejectRight = true;
        }
    }
    else doRejectRight = true;


    if(!doRejectUpdate)
    {
        if(!doRejectLeft && SmartDashboard.getBoolean("LL-Left_Enabled", false))
        {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));

            drivetrain.addVisionMeasurement(
            limelightMeasurementLeft.pose,
            limelightMeasurementLeft.timestampSeconds
            );
        }

        if(!doRejectRight && SmartDashboard.getBoolean("LL-Right_Enabled", false))
        {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
            drivetrain.addVisionMeasurement(
            limelightMeasurementRight.pose,
            limelightMeasurementRight.timestampSeconds
            );
        }
    }
    }

    public void setImuMode1()
    {
        double robotYaw = drivetrain.getGyroHeading();


        LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-left", 1);
  
        
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-right", 1);
    }

    public void setImuMode2()
    {
        double robotYaw = drivetrain.getGyroHeading();


        LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-left", 2);
  
        
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-right", 2);
    }


    public boolean hasAnEstimate() {
        return (currentPoseEstimateFinal != null);
    }
    public double getLimelightTagHeading() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal.pose.getRotation().getDegrees();}
        return 0;
    }

    public Pose2d getLimelightPose() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal.pose;}
        return new Pose2d();
    }

    public LimelightHelpers.PoseEstimate getLimelightPoseEstimate() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal;}
        return new LimelightHelpers.PoseEstimate();
    }

    public Command useMT2() {
        return new InstantCommand(()->{useMT2 = true;});
    }

    
    public Command useMT() {
        return new InstantCommand(()->{useMT2 = false;});
    }

    public void disabledPeriodic()
    {
        setImuMode1();
        isMode1Set = true;
    }

    public void enabledPeriodic()
    {   
        if(!isLLReady && isMode1Set)
        {
            setImuMode2();
            isLLReady = true;
        }

        if(isLLReady) addVisionMeasurement();
    }
}