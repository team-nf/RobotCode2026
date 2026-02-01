// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Feeder.Utils;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Utils.States.FeederStates.FeederControlState;
import frc.robot.Utils.States.FeederStates.FeederRollerState;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class FeederControlData implements Sendable{

    public FeederRollerState feederRollerState = FeederRollerState.ZERO;
    public FeederControlState feederControlState = FeederControlState.ZERO;
    public FeederControlState prevFeederControlState = FeederControlState.ZERO;

    public AngularVelocity feederGoalVelocity = RotationsPerSecond.of(0);
    public AngularVelocity feederVelocity = RotationsPerSecond.of(0);
    public AngularVelocity feederError = RotationsPerSecond.of(0);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FeederControlData");
        builder.addStringProperty("FeederControlState", () -> feederControlState.toString(), null);
        builder.addStringProperty("FeederPrevControlState", () -> prevFeederControlState.toString(), null);
        builder.addStringProperty("FeederRollerState", () -> feederRollerState.toString(), null);
        builder.addDoubleProperty("FeederGoalRPM", () -> feederGoalVelocity.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("FeederRPM", () -> feederVelocity.in(RotationsPerSecond)*60, null);
        builder.addDoubleProperty("FeederErrorRPM", () -> feederError.in(RotationsPerSecond)*60, null);
    }
}