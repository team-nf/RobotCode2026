package frc.robot.Subsytems.TheMachine.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.States.TheMachineStates.TheMachineControlState;

public class TheMachineIdleDeployedRequest extends InstantCommand {

    private final TheMachine theMachine;

    public TheMachineIdleDeployedRequest(TheMachine theMachine) {
        this.theMachine = theMachine;
    }

    @Override
    public void initialize() {
        theMachine.setState(TheMachineControlState.IDLE_DEPLOYED);
    }
}