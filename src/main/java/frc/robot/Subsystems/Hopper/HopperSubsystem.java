package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.Hardware.HopperHardware;
import frc.robot.subsystems.Hopper.Hardware.HopperRealHardware;
import frc.robot.subsystems.Hopper.Hardware.HopperSimHardware;
import frc.robot.subsystems.Hopper.StateActions.*;
import frc.robot.subsystems.Hopper.StateRequests.*;
import frc.robot.subsystems.Hopper.Utils.HopperControlData;
import frc.robot.constants.States.HopperStates;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.TelemetryConstants;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class HopperSubsystem extends SubsystemBase {

  public final HopperHardware hopperHardware;
  public final HopperControlData hopperData;

  public final HopperZeroAction hopperZeroAction;
  public final HopperFeedAction hopperFeedAction;
  public final HopperReverseAction hopperReverseAction;
  public final HopperTestAction hopperTestAction;

  public HopperSubsystem() {
    if (Utils.isSimulation()) {
        hopperHardware = new HopperSimHardware();
    } else {
        hopperHardware = new HopperRealHardware();
    }

    hopperData = new HopperControlData();

    hopperZeroAction = new HopperZeroAction(this);
    hopperFeedAction = new HopperFeedAction(this);
    hopperReverseAction = new HopperReverseAction(this);
    hopperTestAction = new HopperTestAction(this);
  }

  public void updateHopperData() {
    hopperData.hopperVelocity = hopperHardware.getHopperVelocity();
    hopperData.hopperError = hopperData.hopperGoalVelocity.minus(hopperData.hopperVelocity);
    
    hopperData.hopperRollerState = RotationsPerSecond.of(hopperData.hopperError.abs(RotationsPerSecond))
    .lte(HopperConstants.HOPPER_ALLOWABLE_ERROR)
        ? HopperStates.HopperRollerState.AT_SPEED
        : HopperStates.HopperRollerState.REACHING_SPEED;
  }

  public void zero() {
    hopperData.hopperGoalVelocity = RotationsPerSecond.of(0);
    updateHopperData();
    hopperHardware.hopperStop();
  }

  public void feed() {
    hopperData.hopperGoalVelocity = HopperConstants.HOPPER_FEEDING_VELOCITY;
    updateHopperData();
    hopperHardware.setHopperSpeed(hopperData.hopperGoalVelocity);
  }

  public void reverse() {
    hopperData.hopperGoalVelocity = HopperConstants.HOPPER_REVERSE_VELOCITY;
    updateHopperData();
    hopperHardware.setHopperSpeed(hopperData.hopperGoalVelocity);
  }

  public void test() {
    updateHopperData();
    hopperHardware.testHopper();
  }

  public boolean isHopperState(HopperStates.HopperControlState state) {
    return hopperData.hopperControlState == state;
  }

  public boolean isPrevHopperState(HopperStates.HopperControlState state) {
    return hopperData.prevHopperControlState == state;
  }

  public boolean isHopperAtSpeed() {
    return hopperData.hopperRollerState == HopperStates.HopperRollerState.AT_SPEED;
  }

  public void setHopperState(HopperStates.HopperControlState newState) {
    hopperData.prevHopperControlState = hopperData.hopperControlState;
    hopperData.hopperControlState = newState;
  }

  public InstantCommand zeroRequest() {
    return new HopperZeroRequest(this);
  }

  public InstantCommand feedRequest() {
    return new HopperFeedRequest(this);
  }

  public InstantCommand reverseRequest() {
    return new HopperReverseRequest(this);
  }

  public InstantCommand testRequest() {
    return new HopperTestRequest(this);
  }

  public HopperControlData getHopperData() {
    return hopperData;
  }

  public void stateMachine() {
    switch (hopperData.hopperControlState) {
      case ZERO:
        CommandScheduler.getInstance().schedule(hopperZeroAction);
        break;
      case FEEDING:
        CommandScheduler.getInstance().schedule(hopperFeedAction);
        break;
      case REVERSE:
        CommandScheduler.getInstance().schedule(hopperReverseAction);
        break;
      case TEST:
        CommandScheduler.getInstance().schedule(hopperTestAction);
        break;
      default:
        CommandScheduler.getInstance().schedule(hopperZeroAction);
        break;
    }
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
      hopperHardware.update();
    }
    
    if (TelemetryConstants.SHOULD_HOPPER_HARDWARE_COMMUNICATE) {
        SmartDashboard.putData("Hopper Telemetry", hopperHardware);
    }
    if (TelemetryConstants.SHOULD_HOPPER_CONTROL_COMMUNICATE) {
        SmartDashboard.putData("Hopper Control Data", hopperData);
    }

    stateMachine();
  }

  @Override
  public void simulationPeriodic() {
    if (Utils.isSimulation()) {
      hopperHardware.update();
    }
  }
}
