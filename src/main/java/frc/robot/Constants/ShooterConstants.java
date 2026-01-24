package frc.robot.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import java.net.http.HttpResponse.PushPromiseHandler;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.AngularMomentumUnit;
import edu.wpi.first.units.measure.*;

public class ShooterConstants {
    // Motor IDs
    public static final int FIRST_SHOOTER_MOTOR_ID = 31;
    public static final int SECOND_SHOOTER_MOTOR_ID = 32;
    public static final int THIRD_SHOOTER_MOTOR_ID = 33;
    public static final int FOURTH_SHOOTER_MOTOR_ID = 34;
    public static final int HOOD_MOTOR_ID = 35;

    // Configs
    public static final int NUMBER_OF_FLYWHEEL_MOTORS = 4;

    public static final double SHOOTER_KS = 0.24;
    public static final double SHOOTER_KV = 0.12;    
    public static final double SHOOTER_KP = 0.5;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0; 


    public static final MotorOutputConfigs shooterOutputConfig = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    public static final Slot0Configs shooterPIDConfigs = new Slot0Configs()
        .withKS(SHOOTER_KS)
        .withKV(SHOOTER_KV)
        .withKP(SHOOTER_KP)
        .withKI(SHOOTER_KI)
        .withKD(SHOOTER_KD);

    public static final VoltageConfigs shooterVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(8)
        .withPeakReverseVoltage(-8);

    public static final CurrentLimitsConfigs shooterCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(20)
        .withStatorCurrentLimit(20);

    public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
            .withSlot0(shooterPIDConfigs)
            .withVoltage(shooterVoltageConfigs)
            .withCurrentLimits(shooterCurrentLimits);

    public static final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0)
                                                                        .withSlot(0)
                                                                        .withEnableFOC(false);

    public static final double HOOD_KS = 0.0;
    public static final double HOOD_KV = 0.0;
    public static final double HOOD_KP = 0.5;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 0.0;
    public static final double HOOD_KG = 0.1;

    public static final Slot0Configs hoodPIDConfigs = new Slot0Configs()
        .withKS(HOOD_KS)
        .withKV(HOOD_KV)
        .withKP(HOOD_KP)
        .withKI(HOOD_KI)
        .withKD(HOOD_KD)
        .withKG(HOOD_KG);

    public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration()
            .withSlot0(hoodPIDConfigs)
            .withVoltage(shooterVoltageConfigs)
            .withCurrentLimits(shooterCurrentLimits);

    public static final PositionVoltage hoodPositionControl = new PositionVoltage(0)
                                                                    .withSlot(0)
                                                                    .withEnableFOC(false);


    // Physical Constants
    public static final double FLYWHEEL_GEAR_REDUCTION = 1.5;
    public static final double HOOD_GEAR_REDUCTION = 10.0;

    private static final Mass FLYWHEEL_MASS = Kilogram.of(0.16 + 0.05*2); 
    private static final Distance FLYWHEEL_RADIUS = Meters.of((4*2.54)/2/100); // Radius of the flywheel in meters
    private static final Mass ROLLER_MASS = Kilogram.of(15*20/1000); // Mass of the hood in kg
    private static final Distance ROLLER_RADIUS = Meters.of((2*2.54)/2/100); // Radius of the hood in meters

    public static final MomentOfInertia FLYWHEEL_MOMENT_OF_INERTIA = 
                KilogramSquareMeters.of(2*0.5*FLYWHEEL_MASS.in(Kilogram)*Math.pow(FLYWHEEL_RADIUS.in(Meters), 2));
            
    public static final MomentOfInertia HOOD_MOMENT_OF_INERTIA = 
                KilogramSquareMeters.of(2*0.5*ROLLER_MASS.in(Kilogram)*Math.pow(ROLLER_RADIUS.in(Meters), 2));
    public static final MomentOfInertia TOTAL_MOMENT_OF_INERTIA = FLYWHEEL_MOMENT_OF_INERTIA.plus(HOOD_MOMENT_OF_INERTIA);


    public static final Angle MIN_HOOD_ANGLE = Degrees.of(0);
    public static final Angle MAX_HOOD_ANGLE = Degrees.of(20);

    public static final Mass HOOD_MASS = Kilogram.of(0.5);
    public static final Distance HOOD_LENGTH = Meters.of(0.075);
    public static final Distance HOOD_CENTER_OF_MASS = HOOD_LENGTH.times(0.5);
    public static final MomentOfInertia HOOD_INERTIA = 
        KilogramSquareMeters.of(HOOD_MASS.in(Kilogram) * Math.pow(HOOD_LENGTH.in(Meters), 2) / 3.0);


}