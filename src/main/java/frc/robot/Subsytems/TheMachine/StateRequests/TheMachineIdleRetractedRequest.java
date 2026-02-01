package frc.robot.Subsytems.TheMachine.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.States.TheMachineStates.TheMachineControlState;

public class TheMachineIdleRetractedRequest extends InstantCommand {

    private final TheMachine theMachine;

    public TheMachineIdleRetractedRequest(TheMachine theMachine) {
        this.theMachine = theMachine;
    }

    @Override
    public void initialize() {
        theMachine.setState(TheMachineControlState.IDLE_RETRACTED);
    }
}