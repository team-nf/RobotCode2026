// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Feeder.Utils;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.States.FeederStates.FeederControlState;
import frc.robot.Constants.States.FeederStates.FeederRollerState;
import frc.robot.Constants.TelemetryConstants;
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
        builder.addDoubleProperty("FeederGoalRPM", () -> TelemetryConstants.roundTelemetry(feederGoalVelocity.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("FeederRPM", () -> TelemetryConstants.roundTelemetry(feederVelocity.in(RotationsPerSecond) * 60), null);
        builder.addDoubleProperty("FeederErrorRPM", () -> TelemetryConstants.roundTelemetry(feederError.in(RotationsPerSecond) * 60), null);
    }
}