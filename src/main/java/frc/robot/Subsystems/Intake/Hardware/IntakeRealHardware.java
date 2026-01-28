package frc.robot.subsystems.Intake.Hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.constants.IntakeConstants;

public class IntakeRealHardware implements IntakeHardware {

    protected TalonFX intakeMotor;
    protected TalonFX intakeArmMotor;

    private TalonFXConfiguration intakeMotorConfig;
    private TalonFXConfiguration intakeArmMotorConfig;

    private final VelocityVoltage intakeVelocityControl;
    private final PositionVoltage intakeArmPositionControl;

    private AngularVelocity intakeVelocity = RotationsPerSecond.of(0);
    private AngularVelocity intakeMotorVelocity = RotationsPerSecond.of(0);
    private Voltage intakeVoltage = Volts.of(0);
    private Current intakeCurrent = Amps.of(0);
    private double intakeReference = 0.0;
    private double intakeError = 0.0;
    private AngularVelocity testIntakeGoal = RotationsPerSecond.of(0);

    private Angle intakeArmPosition = Degrees.of(0);
    private Angle intakeArmMotorPosition = Degrees.of(0);
    private Voltage intakeArmMotorVoltage = Volts.of(0);
    private Current intakeArmMotorCurrent = Amps.of(0);
    private double intakeArmReference = 0.0;
    private double intakeArmError = 0.0;
    private Angle testIntakeArmGoal = Degrees.of(0);

    public IntakeRealHardware() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

        // No separate arm motor configured in constants by default. Use the intake motor as a
        // fallback for arm telemetry/control when a dedicated motor isn't supplied.
        intakeArmMotor = intakeMotor;

        setIntakeMotorConfig(IntakeConstants.INTAKE_MOTOR_CONFIG);
        // Arm motor config falls back to the intake motor config if a dedicated one isn't present
        setIntakeArmMotorConfig(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
        updateMotorConfig();

        intakeVelocityControl = IntakeConstants.INTAKE_VELOCITY_CONTROL.clone();
        // Use a sensible default PositionVoltage for arm control if none provided
        intakeArmPositionControl = new PositionVoltage(0);
    }
    @Override
    public void updateMotorConfig() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(intakeMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeArmMotor.getConfigurator().apply(intakeArmMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void setIntakeMotorConfig(TalonFXConfiguration config) {
        intakeMotorConfig = config.clone();
    }

    @Override
    public void setIntakeArmMotorConfig(TalonFXConfiguration config) {
        intakeArmMotorConfig = config.clone();
    }

    @Override
    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }

    @Override
    public TalonFX getIntakeArmMotor() {
        return intakeArmMotor;
    }

    @Override
    public AngularVelocity getIntakeVelocity() {
        return intakeVelocity;
    }

    @Override
    public Angle getIntakeArmPosition() {
        return intakeArmPosition;
    }

    @Override
    public void intakeSetPower(double power) {
        intakeMotor.set(power);
    }

    @Override
    public void setIntakeSpeed(AngularVelocity velocity) {
        intakeMotor.setControl(
            intakeVelocityControl.withVelocity(velocity.times(IntakeConstants.INTAKE_GEAR_REDUCTION))
        );
    }

    @Override
    public void intakeStop() {
        intakeMotor.set(0.0);
    }

    @Override
    public void testIntake() {
        setIntakeSpeed(testIntakeGoal);
    }

    @Override
    public void intakeArmSetPower(double power) {
        intakeArmMotor.set(power);
    }

    @Override
    public void setIntakeArmPosition(Angle position) {
        intakeArmMotor.setControl(
            intakeArmPositionControl.withPosition(position)
        );
        intakeArmPosition = position;
    }

    @Override
    public void intakeArmZero() {
        setIntakeArmPosition(IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE);
    }

    @Override
    public void testArmIntake() {
        setIntakeArmPosition(testIntakeArmGoal);
    }

    @Override
    public void updateVariables() {
        intakeVelocity = intakeMotor.getVelocity().getValue().div(IntakeConstants.INTAKE_GEAR_REDUCTION);
        intakeMotorVelocity = intakeMotor.getVelocity().getValue();
        intakeVoltage = intakeMotor.getMotorVoltage().getValue();
        intakeCurrent = intakeMotor.getStatorCurrent().getValue();
        intakeReference = intakeMotor.getClosedLoopReference().getValue();
        intakeError = intakeMotor.getClosedLoopError().getValue();

        intakeArmPosition = intakeArmMotor.getRotorPosition().getValue();
        intakeArmMotorPosition = intakeArmMotor.getRotorPosition().getValue();
        intakeArmMotorVoltage = intakeArmMotor.getMotorVoltage().getValue();
        intakeArmMotorCurrent = intakeArmMotor.getStatorCurrent().getValue();
        intakeArmReference = intakeArmMotor.getClosedLoopReference().getValue();
        intakeArmError = intakeArmMotor.getClosedLoopError().getValue();
    }

    @Override
    public void update() {
        updateVariables();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("IntakeHardware");

        builder.addDoubleProperty("Intake Velocity (RPM)", () -> intakeVelocity.times(60).in(RotationsPerSecond), null);
        builder.addDoubleProperty("Intake Motor Velocity (RPM)", () -> intakeMotorVelocity.times(60).in(RotationsPerSecond), null);
        builder.addDoubleProperty("Intake Voltage (V)", () -> intakeVoltage.in(Volts), null);
        builder.addDoubleProperty("Intake Current (A)", () -> intakeCurrent.in(Amps), null);
        builder.addDoubleProperty("Intake Reference", () -> intakeReference, null);
        builder.addDoubleProperty("Intake Error", () -> intakeError, null);
        builder.addDoubleProperty("Intake Arm Position (deg)", () -> intakeArmPosition.in(Degrees), null);
        builder.addDoubleProperty("Intake Arm Motor Position (deg)", () -> intakeArmMotorPosition.in(Degrees), null);
        builder.addDoubleProperty("Intake Arm Motor Voltage (V)", () -> intakeArmMotorVoltage.in(Volts), null);
        builder.addDoubleProperty("Intake Arm Motor Current (A)", () -> intakeArmMotorCurrent.in(Amps), null);
        builder.addDoubleProperty("Intake Arm Reference", () -> intakeArmReference, null);
        builder.addDoubleProperty("Intake Arm Error", () -> intakeArmError, null);

        builder.addDoubleProperty("Test Intake Goal RPM", () -> testIntakeGoal.times(60).in(RotationsPerSecond),
            (value) -> testIntakeGoal = RotationsPerSecond.of(value / 60.0));
        
        builder.addDoubleProperty("Test Intake Arm Goal Degrees", () -> testIntakeArmGoal.in(Degrees),
            (value) -> testIntakeArmGoal = Degrees.of(value));

        // Allow tuning of the arm PID/FF slot0 values from the dashboard
        builder.addDoubleProperty("Intake Arm KS", () -> intakeArmMotorConfig.Slot0.kS, (value) -> {
            intakeArmMotorConfig.Slot0.kS = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KV", () -> intakeArmMotorConfig.Slot0.kV, (value) -> {
            intakeArmMotorConfig.Slot0.kV = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KP", () -> intakeArmMotorConfig.Slot0.kP, (value) -> {
            intakeArmMotorConfig.Slot0.kP = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KI", () -> intakeArmMotorConfig.Slot0.kI, (value) -> {
            intakeArmMotorConfig.Slot0.kI = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KD", () -> intakeArmMotorConfig.Slot0.kD, (value) -> {
            intakeArmMotorConfig.Slot0.kD = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KS", () -> intakeMotorConfig.Slot0.kS, (value) -> {
            intakeMotorConfig.Slot0.kS = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KV", () -> intakeMotorConfig.Slot0.kV, (value) -> {
            intakeMotorConfig.Slot0.kV = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KP", () -> intakeMotorConfig.Slot0.kP, (value) -> {
            intakeMotorConfig.Slot0.kP = value;
            updateMotorConfig();
        });
        
        builder.addDoubleProperty("Intake KI", () -> intakeMotorConfig.Slot0.kI, (value) -> {
            intakeMotorConfig.Slot0.kI = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KD", () -> intakeMotorConfig.Slot0.kD, (value) -> {
            intakeMotorConfig.Slot0.kD = value;
            updateMotorConfig();
        });

        
    }
}
