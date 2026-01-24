package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class SimShooterHardware extends RealShooterHardware {

    // Simulations
    private final DCMotor shDcMotor;
    private final LinearSystem<N2, N1, N2> shFlywheelSystem;
    private final DCMotorSim shFlywheelSim;

    private final DCMotor hoodDcMotor;
    private final SingleJointedArmSim hoodSim;

    private boolean isSimulationInitialized = false;

    public SimShooterHardware() {
        super();

        // Flywheel systems and sims
        shDcMotor = DCMotor.getKrakenX60(ShooterConstants.NUMBER_OF_FLYWHEEL_MOTORS);
        shFlywheelSystem = LinearSystemId.createDCMotorSystem(
            shDcMotor,
            ShooterConstants.FLYWHEEL_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            ShooterConstants.FLYWHEEL_GEAR_REDUCTION);

        shFlywheelSim = new DCMotorSim(shFlywheelSystem, shDcMotor);

        // Hood systems and sims
        hoodDcMotor = DCMotor.getKrakenX60(1);
        hoodSim = new SingleJointedArmSim(
            hoodDcMotor,
            ShooterConstants.HOOD_GEAR_REDUCTION,
            ShooterConstants.HOOD_INERTIA.in(KilogramSquareMeters), 
            ShooterConstants.HOOD_LENGTH.in(Meters),
            ShooterConstants.MIN_HOOD_ANGLE.in(Radian), 
            ShooterConstants.MAX_HOOD_ANGLE.in(Radian), 
            false, 
            0, 0.0002, 0.0002);
    }

    @Override
    public void update() {
        super.update();
        if (!isSimulationInitialized) {
            isSimulationInitialized = true;
            firstShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            firstShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            secondShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            secondShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            thirdShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            thirdShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            fourthShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            fourthShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            hoodMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            hoodMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
            final var firstShootSimState = firstShootMotor.getSimState();
            final var secondShootSimState = secondShootMotor.getSimState();
            final var thirdShootSimState = thirdShootMotor.getSimState();
            final var fourthShootSimState = fourthShootMotor.getSimState();
            final var hoodSimState = hoodMotor.getSimState();

            /* First set the supply voltage of all the devices */
            firstShootSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            secondShootSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            thirdShootSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            fourthShootSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            /* Then calculate the new velocity of the simulated flywheel */
            shFlywheelSim.setInputVoltage(firstShootSimState.getMotorVoltage());
            shFlywheelSim.update(0.002);

            /* Apply the new rotor velocity to the motors (before gear ratio) */
            firstShootSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            secondShootSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            thirdShootSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fourthShootSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            

            firstShootSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            secondShootSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            thirdShootSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fourthShootSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            /* Now do the same for the hood */
            hoodSim.setInput(hoodSimState.getMotorVoltage());
            hoodSim.update(0.002);

            hoodSimState.setRawRotorPosition(
                Radians.of(hoodSim.getAngleRads()).times(ShooterConstants.HOOD_GEAR_REDUCTION));

            hoodSimState.setRotorVelocity(
                RotationsPerSecond.of(hoodSim.getVelocityRadPerSec()).times(ShooterConstants.HOOD_GEAR_REDUCTION));

        }

    }

}
