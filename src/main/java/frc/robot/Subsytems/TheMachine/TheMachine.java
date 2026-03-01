// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.TheMachine;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.TheMachineConstants;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.Feeder.FeederSubsystem;
import frc.robot.Subsytems.Hopper.HopperSubsystem;
import frc.robot.Subsytems.Intake.IntakeSubsystem;
import frc.robot.Subsytems.Shooter.ShooterSubsystem;
import frc.robot.Subsytems.Swerve.Utils.SwerveControlData;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineGetReadyAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineIdleAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineIdleDeployedAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineIdleRetractedAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineIntakeAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineReverseAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineShootAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineTestAction;
import frc.robot.Subsytems.TheMachine.StateActions.TheMachineZeroAction;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineGetReadyRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineIdleDeployedRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineIdleRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineIdleRetractedRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineIntakeRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineIntakeToggledRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineNoneRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachinePreviousStateRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineReverseRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineShootRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineTestRequest;
import frc.robot.Subsytems.TheMachine.StateRequests.TheMachineZeroRequest;
import frc.robot.Subsytems.TheMachine.Utils.LEDController;
import frc.robot.Subsytems.TheMachine.Utils.TheMachineControlData;

public class TheMachine {
  /** Creates a new TheMachine. */

  private TheMachineControlData theMachineData;

  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private HopperSubsystem hopperSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private Supplier<SwerveControlData> swerveDataSupplier;

  private Command theMachineZeroAction;
  private Command theMachineIdleDeployedAction;
  private Command theMachineIdleRetractedAction;
  private Command theMachineIntakeAction;
  private Command theMachineShootAction;
  private Command theMachineReverseAction;
  private Command theMachineTestAction;
  private Command theMachineIdleAction;
  private Command theMachineGetReadyAction;

  private LEDController leftLed;
  //private LEDController rightLed;
  //private LEDController frontLed;
  //private LEDController backLed;

  public TheMachine(
    ShooterSubsystem shooterSubsystem,
    FeederSubsystem feederSubsystem,
    HopperSubsystem hopperSubsystem,
    IntakeSubsystem intakeSubsystem,
    Supplier<SwerveControlData> swerveDataSupplier
  ) 
  {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.swerveDataSupplier = swerveDataSupplier;

    theMachineData = new TheMachineControlData(shooterSubsystem.getShooterData(), 
                                              feederSubsystem.getFeederData(), 
                                              hopperSubsystem.getHopperData(), 
                                              intakeSubsystem.getIntakeData());


    theMachineZeroAction = TheMachineZeroAction.get(this);
    theMachineIdleDeployedAction = TheMachineIdleDeployedAction.get(this);
    theMachineIdleRetractedAction = TheMachineIdleRetractedAction.get(this);
    theMachineIntakeAction = TheMachineIntakeAction.get(this);
    theMachineShootAction = TheMachineShootAction.get(this);
    theMachineReverseAction = TheMachineReverseAction.get(this);
    theMachineTestAction = TheMachineTestAction.get(this);
    theMachineIdleAction = TheMachineIdleAction.get(this);
    theMachineGetReadyAction = TheMachineGetReadyAction.get(this);

    leftLed = new LEDController(0, 30);
    //rightLed = new LEDController(1, 31);
    //frontLed = new LEDController(2,31);
    //backLed = new LEDController(3,31);
  }

  public TheMachineControlState getState() {
    return theMachineData.theMachineControlState;
  }

  public TheMachineControlState getPreviousState() {
    return theMachineData.previousTheMachineControlState;
  }

  public void setState(TheMachineControlState newState) {
    if (theMachineData.theMachineControlState != newState) {
      theMachineData.previousTheMachineControlState = theMachineData.theMachineControlState;
      theMachineData.theMachineControlState = newState;
    }
  }

  public boolean isState(TheMachineControlState state) {
    return theMachineData.theMachineControlState == state;
  }

  // Shooter State Requests

  public InstantCommand shooterZeroRequest() {
    return shooterSubsystem.zeroRequest();
  }

  public InstantCommand shooterRestRequest() {
    return shooterSubsystem.restRequest();
  }

  public InstantCommand shooterShootRequest() {
    return shooterSubsystem.shootRequest();
  }

  public InstantCommand shooterTestRequest() {
    return shooterSubsystem.testRequest();
  }

  // Feeder State Requests

  public InstantCommand feederZeroRequest() {
    return feederSubsystem.zeroRequest();
  }

  public InstantCommand feederFeedRequest() {
    return feederSubsystem.feedRequest();
  }

  public InstantCommand feederReverseRequest() {
    return feederSubsystem.reverseRequest();
  }

  public InstantCommand feederTestRequest() {
    return feederSubsystem.testRequest();
  }

  // Hopper State Requests

  public InstantCommand hopperZeroRequest() {
    return hopperSubsystem.zeroRequest();
  }

  public InstantCommand hopperFeedRequest() {
    return hopperSubsystem.feedRequest();
  }

  public InstantCommand hopperPushRequest() {
    return hopperSubsystem.pushRequest();
  }

  public InstantCommand hopperReverseRequest() {
    return hopperSubsystem.reverseRequest();
  }

  public InstantCommand hopperTestRequest() {
    return hopperSubsystem.testRequest();
  }

  // Intake State Requests

  public InstantCommand intakeCloseRequest() {
    return intakeSubsystem.closeRequest();
  }

  public InstantCommand intakeDeployRequest() {
    return intakeSubsystem.deployRequest();
  }

  public InstantCommand intakeIntakeRequest() {
    return intakeSubsystem.intakeRequest();
  }

  public InstantCommand intakeFeedRequest() {
    return intakeSubsystem.feedRequest();
  }

  public InstantCommand intakeReverseRequest() {
    return intakeSubsystem.reverseRequest();
  }

  public InstantCommand intakeIdleBetweenRequest() {
    return intakeSubsystem.idleBetweenRequest();
  }

  public InstantCommand intakeTestRequest() {
    return intakeSubsystem.testRequest();
  }

  // Wait for Commands
  public WaitUntilCommand waitForShooter()
  {
    return shooterSubsystem.waitForShooterToBeReady();
  }

  public WaitUntilCommand waitForIntakeDeploy()
  {
    return intakeSubsystem.waitForIntakeToBeDeployed();
  }

  public WaitUntilCommand waitForIntakeRetract()
  {
    return intakeSubsystem.waitForIntakeToBeRetracted();
  }

  public Command previousStateRequest() {
    return new TheMachinePreviousStateRequest(this);
  }

  public Command zeroRequest(){
      return new TheMachineZeroRequest(this);
  } 

  public Command idleDeployedRequest(){
      return new TheMachineIdleDeployedRequest(this);
  }
  
  public Command idleRetractedRequest() {
      return new TheMachineIdleRetractedRequest(this);
  }

  public Command idleRequest() {
    return new TheMachineIdleRequest(this);
  }

  public Command intakeRequest() {
      return new TheMachineIntakeRequest(this);
  }

    public InstantCommand intakeToggledRequest() {
      return new TheMachineIntakeToggledRequest(this);
  }

  public Command shootRequest() {
      return new TheMachineShootRequest(this);
  }

  public Command getReadyRequest(){
    return new TheMachineGetReadyRequest(this);
  }

  public Command reverseRequest() {
      return new TheMachineReverseRequest(this);
  }

  public Command testRequest() {
      return new TheMachineTestRequest(this);
  }

  public Command noneRequest() {
    return new TheMachineNoneRequest(this);
  }

  private StructPublisher<Pose3d> hoodPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/HoodPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> funnelPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/FunnelPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> intakePosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/IntakePose", Pose3d.struct).publish();

  public void calculateSubsytemPoses() {

    double hoodAngle = shooterSubsystem.getHoodAngleInRealLife();

    Pose3d hoodPose = TheMachineConstants.HOOD_RETRACTED_POSE
                            .rotateAround(TheMachineConstants.HOOD_RETRACTED_POSE.getTranslation(),new Rotation3d(0, Math.toRadians(hoodAngle*360), 0));

    double intakeArmAngle = intakeSubsystem.getArmAngleInRealLife();

    Pose3d intakePose = TheMachineConstants.INTAKE_DEPLOYED_POSE
                          .rotateAround(TheMachineConstants.INTAKE_DEPLOYED_POSE.getTranslation(), new Rotation3d(0, -Math.toRadians(intakeArmAngle*360), 0));

    Distance funnelExtension = Meters.of(0.0);

    if(intakeArmAngle < 30) funnelExtension = Meters.of(0.3125);
    else funnelExtension = Meters.of(0.3125).times(Math.cos(Math.toRadians(intakeArmAngle - 30)));
    
    Pose3d funnelPose = TheMachineConstants.FUNNEL_RETRACTED_POSE
                          .plus(new Transform3d(funnelExtension.in(Meters), 0, 0, new Rotation3d(0, 0, 0)));
    
    hoodPosePublisher.set(hoodPose);
    intakePosePublisher.set(intakePose);
    funnelPosePublisher.set(funnelPose);
  }

  public void stateMachine() {
    switch (theMachineData.theMachineControlState) {
      case ZERO:
        if(!CommandScheduler.getInstance().isScheduled(theMachineZeroAction))
        {
          CommandScheduler.getInstance().schedule(theMachineZeroAction);
        }
        break;
      case IDLE_DEPLOYED:
        if(!CommandScheduler.getInstance().isScheduled(theMachineIdleDeployedAction))
        {
          CommandScheduler.getInstance().schedule(theMachineIdleDeployedAction);
        }
        break;
      case IDLE_RETRACTED:
        if(!CommandScheduler.getInstance().isScheduled(theMachineIdleRetractedAction))
        {
          CommandScheduler.getInstance().schedule(theMachineIdleRetractedAction);
        }
        break;
      case INTAKE:
        if(!CommandScheduler.getInstance().isScheduled(theMachineIntakeAction))
        {
          CommandScheduler.getInstance().schedule(theMachineIntakeAction);
        }
        break;
      case SHOOT:
        if(!CommandScheduler.getInstance().isScheduled(theMachineShootAction))
        {
          CommandScheduler.getInstance().schedule(theMachineShootAction);
        }
        break;
      case REVERSE:
        if(!CommandScheduler.getInstance().isScheduled(theMachineReverseAction))
        {
          CommandScheduler.getInstance().schedule(theMachineReverseAction);
        }
        break;
      case TEST:
        if(!CommandScheduler.getInstance().isScheduled(theMachineTestAction))
        {
          CommandScheduler.getInstance().schedule(theMachineTestAction);
        }
        break;
      case IDLE:
        if (!CommandScheduler.getInstance().isScheduled(theMachineIdleAction)) {
          CommandScheduler.getInstance().schedule(theMachineIdleAction);
        }
        break;
      case GET_READY:
        if (!CommandScheduler.getInstance().isScheduled(theMachineGetReadyAction)) {
          CommandScheduler.getInstance().schedule(theMachineGetReadyAction);
        }
        break;
      default:
        break;
    }
  }

  public void updateLeds()
  {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
    {
      leftLed.setMultiple2(0,0.5, 0, 0, 255);
    }
    else
    {
      leftLed.setMultiple2(0,0.5, 255, 0, 0);
    }

    leftLed.setMultiple2(0.5,1, 0, 255, 0);
  }

  public void periodic() {
    stateMachine();

    //updateLeds();
    
    if (TelemetryConstants.SHOULD_THEMACHINE_DATA_COMMUNICATE)
    {
      SmartDashboard.putData(theMachineData);
    }

    if (TelemetryConstants.SHOULD_THEMACHINE_SIM_POSES_COMMUNICATE)
    {
      calculateSubsytemPoses();
    }
  }
}