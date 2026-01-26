// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class States {

    public static class ShooterStates {
           public enum FlywheelState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }

            public enum HoodState {
                ZERO,
                MOVING_TO_POSITION,
                AT_POSITION
            }

            public enum ShooterControlState {
                ZERO,
                REST,
                SHOOT_PREPARE,
                SHOOT_READY,
                TEST
            }
    }

    public static class FeederStates {
            public enum FeederControlState {
                ZERO,
                FEED,
                REVERSE,
                TEST
            }

            public enum FeederRollerState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }
    }

    public static class HopperStates {
            public enum HopperControlState {
                ZERO,
                FEEDING,
                REVERSE,
                TEST
            }

            public enum HopperRollerState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }
    }

    public static class IntakeStates {
            public enum IntakeControlState {
                CLOSED,
                INTAKING,
                REVERSE,
                FEEDING,
                TEST
            }

            public enum IntakeRollerState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }

            public enum IntakePositionState {
                DEPLOYED,
                RETRACTED,
                BETWEEN
            }
    }

}
