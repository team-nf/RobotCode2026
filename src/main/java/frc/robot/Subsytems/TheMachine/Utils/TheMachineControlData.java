package frc.robot.Subsytems.TheMachine.Utils;

import com.fasterxml.jackson.annotation.Nulls;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsytems.Feeder.Utils.FeederControlData;
import frc.robot.Subsytems.Hopper.Utils.HopperControlData;
import frc.robot.Subsytems.Intake.Utils.IntakeControlData;
import frc.robot.Subsytems.Shooter.Utils.ShooterControlData;

public class TheMachineControlData implements Sendable{
    
    public ShooterControlData shooterControlData = null;
    public FeederControlData feederControlData = null;
    public HopperControlData hopperControlData = null;
    public IntakeControlData intakeControlData = null;

    public TheMachineControlState previousTheMachineControlState = TheMachineControlState.NONE;
    public TheMachineControlState theMachineControlState = TheMachineControlState.NONE;

    public TheMachineControlData(ShooterControlData shooterControlData, FeederControlData feederControlData,
            HopperControlData hopperControlData, IntakeControlData intakeControlData)
    {
        this.shooterControlData = shooterControlData;
        this.feederControlData = feederControlData;
        this.hopperControlData = hopperControlData;
        this.intakeControlData = intakeControlData;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("TheMachineControlState", () -> theMachineControlState.toString(), null);
        builder.addStringProperty("PreviousTheMachineControlState", () -> previousTheMachineControlState.toString(), null);
        builder.addStringProperty("ShooterControlState", () -> shooterControlData.toString(), null);
        builder.addStringProperty("FeederControlState", () -> feederControlData.toString(), null);
        builder.addStringProperty("HopperControlState", () -> hopperControlData.toString(), null);
        builder.addStringProperty("IntakeControlState", () -> intakeControlData.toString(), null);
    }

}