package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;

public interface ShooterHardware extends Sendable{

    public void updateMotorConfigs();
    public void setShootMotorConfig(TalonFXConfiguration config);
    public void setHoodMotorConfig(TalonFXConfiguration config);

    public TalonFX getFirstShootMotor();
    public TalonFX getSecondShootMotor();
    public TalonFX getThirdShootMotor();
    public TalonFX getFourthShootMotor();

    public AngularVelocity getShooterVelocity();

    public void shooterSetPower(double power);
    public void shooterSetVelocity(AngularVelocity velocity);
    public void shooterStop();

    public TalonFX getHoodMotor();

    public Angle getHoodPosition();

    public void hoodSetPosition(Angle hoodAngle);
    public void hoodStop();

    public void setShooter(AngularVelocity velocity, Angle hoodAngle);
    public void testShooter();

    public void updateVariables();
    public void update();
}

