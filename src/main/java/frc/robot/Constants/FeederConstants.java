package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class FeederConstants {
    public static final int FEEDER_MOTOR_ID = 41;

    public static final double FEEDER_KS = 0.1;
    public static final double FEEDER_KV = 0.0;
    public static final double FEEDER_KP = 0.1;
    public static final double FEEDER_KI = 0.0;
    public static final double FEEDER_KD = 0.01;

    public static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(FEEDER_KS)
                .withKV(FEEDER_KV)
                .withKP(FEEDER_KP)
                .withKI(FEEDER_KI)
                .withKD(FEEDER_KD))

            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12)
                .withPeakReverseVoltage(-12))

            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(15)
                .withStatorCurrentLimit(15));

    public static final VelocityVoltage FEEDER_VELOCITY_CONTROL = new VelocityVoltage(0)
                        .withSlot(0)
                        .withEnableFOC(false);

    public static final double FEEDER_GEAR_REDUCTION = 5.0;

    public static final AngularVelocity FEEDER_ALLOWABLE_ERROR = RotationsPerSecond.of(1.0); // in RPS

    public static final AngularVelocity FEEDER_FEEDING_VELOCITY = RotationsPerSecond.of(10); // in RPS
    public static final AngularVelocity FEEDER_REVERSE_VELOCITY = RotationsPerSecond.of(-3.0); // in RPS

    // Feeder syystem constants

    public static final Mass ROLLER_MASS = Kilograms.of(0.2); // kg
    public static final Distance ROLLER_RADIUS = Meters.of(0.015); //
    public static final int ROLLER_AMOUNT = 4;

    public static final MomentOfInertia FEEDER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(
        0.5*ROLLER_MASS.in(Kilogram)*Math.pow(ROLLER_RADIUS.in(Meters), 2)).times(ROLLER_AMOUNT); // kg*m^2

}
