package frc.robot.Subsytems.TheMachine.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.States.TheMachineStates.TheMachineControlState;

public class TheMachineTestRequest extends InstantCommand {

    private final TheMachine theMachine;

    public TheMachineTestRequest(TheMachine theMachine) {
        this.theMachine = theMachine;
    }

    @Override
    public void initialize() {
        theMachine.setState(TheMachineControlState.TEST);
    }
}