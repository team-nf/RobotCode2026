package frc.robot.Subsystems.Shooter.Hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class ShooterRealHardware implements ShooterHardware {

    protected final TalonFX firstShootMotor;
    protected final TalonFX secondShootMotor;
    protected final TalonFX thirdShootMotor;
    protected final TalonFX fourthShootMotor;
    protected final TalonFX hoodMotor;

    private TalonFXConfiguration shootMotorConfig;
    private TalonFXConfiguration hoodMotorConfig;

    private final VelocityVoltage shootVelocityControl;
    private final PositionVoltage hoodPositionControl;

    private AngularVelocity flywheelVelocity = RotationsPerSecond.of(0);
    private AngularVelocity shootVelocity = RotationsPerSecond.of(0);
    private Voltage shootVoltage = Volts.of(0);
    private Current shootCurrent = Amps.of(0);
    private double shootReference = 0.0;
    private double shootError = 0.0;
    private Angle hoodPosition = Degrees.of(0);
    private Angle hoodMotorPosition = Degrees.of(0);
    private Voltage hoodVoltage = Volts.of(0);
    private Current hoodCurrent = Amps.of(0);
    private double hoodReference = 0.0;
    private double hoodError = 0.0;
    private AngularVelocity testFlywheelGoal= RotationsPerSecond.of(0);
    private Angle testHoodGoal = ShooterConstants.MIN_HOOD_ANGLE;

    public ShooterRealHardware() {
        firstShootMotor = new TalonFX(ShooterConstants.FIRST_SHOOTER_MOTOR_ID);
        secondShootMotor = new TalonFX(ShooterConstants.SECOND_SHOOTER_MOTOR_ID);
        thirdShootMotor = new TalonFX(ShooterConstants.THIRD_SHOOTER_MOTOR_ID);
        fourthShootMotor = new TalonFX(ShooterConstants.FOURTH_SHOOTER_MOTOR_ID);

        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

        setShootMotorConfig(ShooterConstants.SHOOTER_CONFIG);
        setHoodMotorConfig(ShooterConstants.HOOD_CONFIG);
        updateMotorConfigs();

        secondShootMotor.setControl(new Follower(firstShootMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        thirdShootMotor.setControl(new Follower(firstShootMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        fourthShootMotor.setControl(new Follower(firstShootMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        shootVelocityControl = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();
        hoodPositionControl = ShooterConstants.HOOD_POSITION_CONTROL.clone();


    }

    @Override
    public void updateMotorConfigs() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = firstShootMotor.getConfigurator().apply(shootMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hoodMotor.getConfigurator().apply(hoodMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void setShootMotorConfig(TalonFXConfiguration config) {
        shootMotorConfig = config.clone();
    }

    @Override
    public void setHoodMotorConfig(TalonFXConfiguration config) {
        hoodMotorConfig = config.clone();
    }

    @Override
    public TalonFX getFirstShootMotor() {
        return firstShootMotor;
    }

    @Override
    public TalonFX getSecondShootMotor() {
        return secondShootMotor;
    }

    @Override
    public TalonFX getThirdShootMotor() {
        return thirdShootMotor;
    }

    @Override
    public TalonFX getFourthShootMotor() {
        return fourthShootMotor;
    }

    @Override
    public AngularVelocity getFlywheelVelocity() {
        // Returns the velocity of the first shooter motor in rotations per second
        return flywheelVelocity;
    }

    @Override
    public void flywheelSetPower(double power) {
        // Sets the power for the first shooter motor (others are followers)
        firstShootMotor.set(power);
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity velocity) {
        // Set velocity in rotations per second
        firstShootMotor.setControl(
            shootVelocityControl.withVelocity(velocity.times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION).in(RotationsPerSecond))
        );
    }

    @Override
    public void flywheelStop() {
        firstShootMotor.set(0.0);
    }

    @Override
    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    @Override
    public Angle getHoodPosition() {
        // Example: Returns the hood position in rotations
        return hoodPosition;
    }

    @Override
    public void setHoodAngle(Angle hoodAngle) {
        // Set hood position in rotations
        hoodMotor.setControl(
            hoodPositionControl.withPosition(hoodAngle.times(ShooterConstants.HOOD_GEAR_REDUCTION).in(Rotations))
        );
    }

    @Override
    public void hoodStop() {
        hoodMotor.set(0.0);
    }

    @Override
    public void setShooter(AngularVelocity velocity, Angle hoodAngle) {
        setFlywheelSpeed(velocity);
        setHoodAngle(hoodAngle);
    }

    @Override
    public void testShooter() {
        setShooter(testFlywheelGoal, testHoodGoal);
    }

    
    @Override
    public void updateVariables() {
        flywheelVelocity = firstShootMotor.getVelocity().getValue().div(ShooterConstants.FLYWHEEL_GEAR_REDUCTION);
        shootVelocity = firstShootMotor.getVelocity().getValue();
        shootVoltage = firstShootMotor.getMotorVoltage().getValue();
        shootCurrent = firstShootMotor.getStatorCurrent().getValue();
        shootReference = firstShootMotor.getClosedLoopReference().getValue();
        shootError = firstShootMotor.getClosedLoopError().getValue();

        hoodPosition = hoodMotor.getPosition().getValue().div(ShooterConstants.HOOD_GEAR_REDUCTION);
        hoodMotorPosition = hoodMotor.getPosition().getValue();
        hoodVoltage = hoodMotor.getMotorVoltage().getValue();
        hoodCurrent = hoodMotor.getStatorCurrent().getValue();
        hoodReference = hoodMotor.getClosedLoopReference().getValue();
        hoodError = hoodMotor.getClosedLoopError().getValue();
    }

    @Override
    public void update() {
        updateVariables();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Flywheel Velocity", () -> flywheelVelocity.times(60).in(RotationsPerSecond), null);
        builder.addDoubleProperty("Shoot Motor Velocity", () -> shootVelocity.times(60).in(RotationsPerSecond), null);
        builder.addDoubleProperty("Shoot Voltage", () -> shootVoltage.in(Volts), null);
        builder.addDoubleProperty("Shoot Current", () -> shootCurrent.in(Amps), null);
        builder.addDoubleProperty("Shoot Reference", () -> shootReference, null);
        builder.addDoubleProperty("Shoot Error", () -> shootError, null);
        builder.addDoubleProperty("Hood Position", () -> hoodPosition.in(Degree), null);
        builder.addDoubleProperty("Hood Motor Position", () -> hoodMotorPosition.in(Degree), null);
        builder.addDoubleProperty("Hood Voltage", () -> hoodVoltage.in(Volts), null);
        builder.addDoubleProperty("Hood Current", () -> hoodCurrent.in(Amps), null);
        builder.addDoubleProperty("Hood Reference", () -> hoodReference, null);
        builder.addDoubleProperty("Hood Error", () -> hoodError*360, null);

        builder.addDoubleProperty("Test Flywheel Goal", () -> testFlywheelGoal.times(60).in(RotationsPerSecond), (val) -> {
            testFlywheelGoal = RotationsPerSecond.of(val/60);
        });

        builder.addDoubleProperty("Test Hood Goal", () -> testHoodGoal.in(Degree), (val) -> {
            testHoodGoal = Degrees.of(val);
        });

        builder.addDoubleProperty("Shoot KS", () -> shootMotorConfig.Slot0.kS, (val) -> {
            shootMotorConfig.Slot0.kS = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KV", () -> shootMotorConfig.Slot0.kV, (val) -> {
            shootMotorConfig.Slot0.kV = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KP", () -> shootMotorConfig.Slot0.kP, (val) -> {
            shootMotorConfig.Slot0.kP = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KI", () -> shootMotorConfig.Slot0.kI, (val) -> {
            shootMotorConfig.Slot0.kI = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KD", () -> shootMotorConfig.Slot0.kD, (val) -> {
            shootMotorConfig.Slot0.kD = val;
            updateMotorConfigs();
        });

        builder.addDoubleProperty("Hood KS", () -> hoodMotorConfig.Slot0.kS, (val) -> {
            hoodMotorConfig.Slot0.kS = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KV", () -> hoodMotorConfig.Slot0.kV, (val) -> {
            hoodMotorConfig.Slot0.kV = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KP", () -> hoodMotorConfig.Slot0.kP, (val) -> {
            hoodMotorConfig.Slot0.kP = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KI", () -> hoodMotorConfig.Slot0.kI, (val) -> {
            hoodMotorConfig.Slot0.kI = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KD", () -> hoodMotorConfig.Slot0.kD, (val) -> {
            hoodMotorConfig.Slot0.kD = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KG", () -> hoodMotorConfig.Slot0.kG, (val) -> {
            hoodMotorConfig.Slot0.kG = val;
            updateMotorConfigs();
        });



    }
}