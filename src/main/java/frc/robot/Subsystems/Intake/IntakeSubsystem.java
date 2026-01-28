package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.subsystems.Intake.Hardware.IntakeHardware;
import frc.robot.subsystems.Intake.Hardware.IntakeRealHardware;
import frc.robot.subsystems.Intake.Hardware.IntakeSimHardware;
import frc.robot.subsystems.Intake.StateActions.*;
import frc.robot.subsystems.Intake.StateRequests.*;
import frc.robot.subsystems.Intake.Utils.IntakeControlData;
import frc.robot.constants.States.IntakeStates;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {

  public final IntakeHardware intakeHardware;
  public final IntakeControlData intakeData;

  public final IntakeClosedAction intakeClosedAction;
  public final IntakeDeployAction intakeDeployAction;
  public final IntakeIntakeAction intakeIntakeAction;
  public final IntakeFeedAction intakeFeedAction;
  public final IntakeReverseAction intakeReverseAction;
  public final IntakeTestAction intakeTestAction;

  public IntakeSubsystem() {
    if (Utils.isSimulation()) {
        intakeHardware = new IntakeSimHardware();
    } else {
        intakeHardware = new IntakeRealHardware();
    }

    intakeData = new IntakeControlData();

  intakeClosedAction = new IntakeClosedAction(this);
    intakeDeployAction = new IntakeDeployAction(this);
    intakeIntakeAction = new IntakeIntakeAction(this);
    intakeFeedAction = new IntakeFeedAction(this);
    intakeReverseAction = new IntakeReverseAction(this);
    intakeTestAction = new IntakeTestAction(this);
  }

  public void updateIntakeData() {
    intakeData.intakeVelocity = intakeHardware.getIntakeVelocity();
    intakeData.intakeError = intakeData.intakeGoalVelocity.minus(intakeData.intakeVelocity);
    
    intakeData.intakeRollerState = RotationsPerSecond.of(intakeData.intakeError.abs(RotationsPerSecond))
    .lte(IntakeConstants.INTAKE_ALLOWABLE_ERROR)
        ? IntakeStates.IntakeRollerState.AT_SPEED
        : IntakeStates.IntakeRollerState.REACHING_SPEED;

    intakeData.intakeArmAngle = intakeHardware.getIntakeArmPosition();
    intakeData.intakeArmError = intakeData.intakeGoalArmAngle.minus(intakeData.intakeArmAngle);

    if(intakeData.intakeArmAngle.lte(IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.minus(IntakeConstants.INTAKE_ARM_ALLOWABLE_ERROR)))
    {
        intakeData.intakePositionState = IntakeStates.IntakePositionState.DEPLOYED;
    }
    else if(intakeData.intakeArmAngle.gte(IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE.plus(IntakeConstants.INTAKE_ARM_ALLOWABLE_ERROR)))
    {
        intakeData.intakePositionState = IntakeStates.IntakePositionState.RETRACTED;
    }
    else
    {
        intakeData.intakePositionState = IntakeStates.IntakePositionState.BETWEEN;
    }
  }

  public void close() {
    intakeData.intakeGoalVelocity = RotationsPerSecond.of(0);
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE;
    updateIntakeData();
    intakeHardware.intakeStop();
    intakeHardware.intakeArmZero();
  }

  public void feed() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY;
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE;
    updateIntakeData();
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.RETRACTED)
        intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);
  }

  public void deploy() {
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE;
    intakeData.intakeGoalVelocity = RotationsPerSecond.of(0);
    updateIntakeData();
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    intakeHardware.intakeStop();
  }

  public void intake() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY;
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE;
    updateIntakeData();
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);

    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.DEPLOYED)
        intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);

  }

  public void reverse() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_REVERSE_VELOCITY;
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE;
    updateIntakeData();
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.DEPLOYED)
        intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);
  }

  public void test() {
    updateIntakeData();
    intakeHardware.testIntake();
    intakeHardware.testArmIntake();
  }

  public boolean isIntakeState(IntakeStates.IntakeControlState state) {
    return intakeData.intakeControlState == state;
  }

  public boolean isPrevIntakeState(IntakeStates.IntakeControlState state) {
    return intakeData.prevIntakeControlState == state;
  }

  public void setIntakeState(IntakeStates.IntakeControlState newState) {
    intakeData.prevIntakeControlState = intakeData.intakeControlState;
    intakeData.intakeControlState = newState;
  }

  public InstantCommand closeRequest() {
    return new IntakeCloseRequest(this);
  }

  public InstantCommand intakeRequest() {
    return new IntakeIntakeRequest(this);
  }

  public InstantCommand feedRequest() {
    return new IntakeFeedRequest(this);
  }

  public InstantCommand reverseRequest() {
    return new IntakeReverseRequest(this);
  }

  public InstantCommand testRequest() {
    return new IntakeTestRequest(this);
  }

  public IntakeControlData getIntakeData() {
    return intakeData;
  }

  public void stateMachine() {
    switch (intakeData.intakeControlState) {
      case CLOSE:
        CommandScheduler.getInstance().schedule(intakeClosedAction);
        break;
      case DEPLOY:
        CommandScheduler.getInstance().schedule(intakeDeployAction);
        break;
      case INTAKE:
        CommandScheduler.getInstance().schedule(intakeIntakeAction);
        break;
      case FEED:
        CommandScheduler.getInstance().schedule(intakeFeedAction);
        break;
      case REVERSE:
        CommandScheduler.getInstance().schedule(intakeReverseAction);
        break;
      case TEST:
        CommandScheduler.getInstance().schedule(intakeTestAction);
        break;
      default:
        CommandScheduler.getInstance().schedule(intakeClosedAction);
        break;
    }   
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
      intakeHardware.update();
    }
    
    if (TelemetryConstants.SHOULD_INTAKE_HARDWARE_COMMUNICATE) {
        SmartDashboard.putData("Intake Telemetry", intakeHardware);
    }
    if (TelemetryConstants.SHOULD_INTAKE_CONTROL_COMMUNICATE) {
        SmartDashboard.putData("Intake Control Data", intakeData);
    }

    stateMachine();
  }

  @Override
  public void simulationPeriodic() {
    if (Utils.isSimulation()) {
      intakeHardware.update();
    }
  }
}
