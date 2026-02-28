// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.States.FeederStates.FeederControlState;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Constants.States.ShooterStates.ShooterControlState;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Subsytems.Feeder.FeederSubsystem;
import frc.robot.Subsytems.Hopper.HopperSubsystem;
import frc.robot.Subsytems.Intake.IntakeSubsystem;
import frc.robot.Subsytems.Shooter.ShooterSubsystem;
import frc.robot.Subsytems.Shooter.Utils.ShooterCalculator;
import frc.robot.Subsytems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsytems.TheMachine.TheMachine;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.HopperSim;
import frc.robot.Utils.MatchTracker;
import frc.robot.Utils.ShooterSim;
import frc.robot.Utils.SwerveFieldContactSim;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private CommandSwerveDrivetrain m_swerveDrivetrain;

  private ShooterCalculator m_shooterCalculator;
  private ShooterSubsystem m_shooterSubsystem;

  private FeederSubsystem m_feederSubsystem;

  
  private HopperSubsystem m_hopperSubsystem;
  
  private IntakeSubsystem m_intakeSubsystem;
  

  private TheMachine m_theMachine;

  private final CommandXboxController m_driverController;

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    m_feederSubsystem = new FeederSubsystem();

    m_hopperSubsystem = new HopperSubsystem();

    m_intakeSubsystem = new IntakeSubsystem();

    m_swerveDrivetrain = TunerConstants.createDrivetrain();

    m_shooterCalculator = new ShooterCalculator(m_swerveDrivetrain.swerveDataSupplier());
    m_shooterSubsystem = new ShooterSubsystem(m_shooterCalculator);

    
    m_theMachine = new TheMachine(
                            m_shooterSubsystem, 
                            m_feederSubsystem, 
                            m_hopperSubsystem, 
                            m_intakeSubsystem, 
                            m_swerveDrivetrain.swerveDataSupplier());
    

    //m_ledSubsytem = new LedSubsytem();

    m_driverController = new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

    configureBindings();
    if(Utils.isSimulation()) configureSims();
    //configureTelemetry();

    boolean isCompetition = false;
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    m_swerveDrivetrain.setDefaultCommand(m_swerveDrivetrain.teleopCommand(m_driverController));

    m_driverController.b()
        .onTrue(m_theMachine.reverseRequest());

    m_driverController.y()
        .whileTrue(m_swerveDrivetrain.aimToHub()
            .alongWith(m_swerveDrivetrain.waitForAtAim().andThen(m_theMachine.shootRequest())))
        .onFalse(m_theMachine.idleDeployedRequest());

    m_driverController.rightBumper()
        .whileTrue(m_theMachine.shootRequest())
        .onFalse(m_theMachine.zeroRequest());

    m_driverController.a()
        .onTrue(m_theMachine.idleRetractedRequest());

    m_driverController.x()
        .onTrue(m_theMachine.intakeRequest().onlyIf(() -> !m_theMachine.isState(TheMachineControlState.INTAKE)))
        .onTrue(m_theMachine.idleDeployedRequest().onlyIf(() -> m_theMachine.isState(TheMachineControlState.INTAKE)));

    m_driverController.povDown()
        .onTrue(m_theMachine.zeroRequest());

    m_driverController.povLeft()
        .onTrue(m_theMachine.noneRequest());

    m_driverController.povUp()
          .onTrue(m_swerveDrivetrain.resetToStartPoseCmd());

    m_driverController.povRight().onTrue(m_shooterSubsystem.testRequest()).onFalse(m_shooterSubsystem.zeroRequest());

    NamedCommands.registerCommand("AimAndShoot", 
          m_swerveDrivetrain.aimToHub()
            .alongWith(m_swerveDrivetrain.waitForAtAim().andThen(m_theMachine.shootRequest()))
            .withTimeout(Seconds.of(5))
            .andThen(m_theMachine.idleRetractedRequest()));

    NamedCommands.registerCommand("MachineIntakeRequest", m_theMachine.intakeRequest());
    NamedCommands.registerCommand("MachineIdleDeployedRequest", m_theMachine.idleDeployedRequest());
    NamedCommands.registerCommand("MachineIdleRetractedRequest", m_theMachine.idleRetractedRequest());
    NamedCommands.registerCommand("MachineIdleRequest", m_theMachine.idleRequest());

    NamedCommands.registerCommand("SetStartPose1", m_swerveDrivetrain.setStartPose1Command());

    NamedCommands.registerCommand("GoToIntakeFromWall", m_swerveDrivetrain.pathFindToIntakeWall());
    NamedCommands.registerCommand("GoToTrench1", m_swerveDrivetrain.pathFindToTrench1());
    NamedCommands.registerCommand("IntakeFromWall", m_theMachine.intakeRequest().andThen(m_swerveDrivetrain.followPathIntakeWall()));
    NamedCommands.registerCommand("IntakeFromTrench1", m_theMachine.intakeRequest().andThen(m_swerveDrivetrain.followPathTrench1()));

    }

    
  private void configureSims() {
    FuelSim fuelSim = FuelSim.getInstance();
    HopperSim hopperSim = HopperSim.getInstance();
    ShooterSim shooterSim = ShooterSim.getInstance();
    MatchTracker matchTracker = MatchTracker.getInstance();

    SwerveFieldContactSim.getInstance().setSwerveDrivetrain(m_swerveDrivetrain);
    SwerveFieldContactSim.getInstance().setIntakeDeployedSupplier(() -> m_intakeSubsystem.isIntakeDeployed());

    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(
            Dimensions.BUMPER_WIDTH.in(Meters),
            Dimensions.BUMPER_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            m_swerveDrivetrain::getPose,
            m_swerveDrivetrain::getFieldSpeeds);
    
    fuelSim.registerIntake(
            Dimensions.BUMPER_LENGTH.div(2).in(Meters),
            Dimensions.BUMPER_LENGTH.div(2).plus(Dimensions.HOPPER_EXTENSION_LENGTH).in(Meters),
            -Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            () -> m_intakeSubsystem.isIntakeDeployed() 
                        && m_intakeSubsystem.isIntakeState(IntakeControlState.INTAKE) 
                        && hopperSim.isHopperAbleToIntake(),
            hopperSim::addFuelToHopper);

    fuelSim.start();

    hopperSim.setRobotPoseSupplier(m_swerveDrivetrain::getPose);
    hopperSim.setShouldRemoveFuelSupplier(() -> m_intakeSubsystem.isIntakeDeployed() 
                        && m_intakeSubsystem.isIntakeState(IntakeControlState.REVERSE));

    shooterSim.setShooterControlDataSupplier(m_shooterSubsystem::getCurrentControlData);
    shooterSim.setRobotPoseSupplier(m_swerveDrivetrain::getPose);
    shooterSim.setChassisSpeedsSupplier(m_swerveDrivetrain::getFieldSpeeds);
    shooterSim.setShouldShootSupplier(() -> m_shooterSubsystem.isShooterState(ShooterControlState.SHOOT) && m_feederSubsystem.isFeederState(FeederControlState.FEED));

    m_driverController.start().onTrue(matchTracker.startMatchCommand());

    FuelSim.BLUE_HUB.setHubActiveSupplier(matchTracker::isBlueHubActive);
    FuelSim.RED_HUB.setHubActiveSupplier(matchTracker::isRedHubActive);
  }

  public void configureTelemetry() {
      SmartDashboard.putBoolean("Telemetry/Settings", false);
      SmartDashboard.putBoolean("Telemetry/Feeder Control", TelemetryConstants.SHOULD_FEEDER_CONTROL_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Feeder Hardware", TelemetryConstants.SHOULD_FEEDER_HARDWARE_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Shooter Control", TelemetryConstants.SHOULD_SHOOTER_CONTROL_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Shooter Hardware", TelemetryConstants.SHOULD_SHOOTER_HARDWARE_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Hopper Control", TelemetryConstants.SHOULD_HOPPER_CONTROL_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Hopper Hardware", TelemetryConstants.SHOULD_HOPPER_HARDWARE_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Intake Control", TelemetryConstants.SHOULD_INTAKE_CONTROL_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Intake Hardware", TelemetryConstants.SHOULD_INTAKE_HARDWARE_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Swerve Data", TelemetryConstants.SHOULD_SWERVE_DATA_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Swerve CTRE", TelemetryConstants.SHOULD_SWERVE_CTRE_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/TheMachine Data", TelemetryConstants.SHOULD_THEMACHINE_DATA_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/TheMachine Sim Poses", TelemetryConstants.SHOULD_THEMACHINE_SIM_POSES_COMMUNICATE);
      SmartDashboard.putBoolean("Telemetry/Scheduler", TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE);  
  }

  public void updateTelemetrySettings() {
    if(SmartDashboard.getBoolean("Telemetry/Settings", false))
    {
      TelemetryConstants.SHOULD_FEEDER_CONTROL_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Feeder Control", TelemetryConstants.SHOULD_FEEDER_CONTROL_COMMUNICATE);
      TelemetryConstants.SHOULD_FEEDER_HARDWARE_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Feeder Hardware", TelemetryConstants.SHOULD_FEEDER_HARDWARE_COMMUNICATE);
      TelemetryConstants.SHOULD_SHOOTER_CONTROL_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Shooter Control", TelemetryConstants.SHOULD_SHOOTER_CONTROL_COMMUNICATE);
      TelemetryConstants.SHOULD_SHOOTER_HARDWARE_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Shooter Hardware", TelemetryConstants.SHOULD_SHOOTER_HARDWARE_COMMUNICATE);
      TelemetryConstants.SHOULD_HOPPER_CONTROL_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Hopper Control", TelemetryConstants.SHOULD_HOPPER_CONTROL_COMMUNICATE);
      TelemetryConstants.SHOULD_HOPPER_HARDWARE_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Hopper Hardware", TelemetryConstants.SHOULD_HOPPER_HARDWARE_COMMUNICATE);
      TelemetryConstants.SHOULD_INTAKE_CONTROL_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Intake Control", TelemetryConstants.SHOULD_INTAKE_CONTROL_COMMUNICATE);
      TelemetryConstants.SHOULD_INTAKE_HARDWARE_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Intake Hardware", TelemetryConstants.SHOULD_INTAKE_HARDWARE_COMMUNICATE);
      TelemetryConstants.SHOULD_SWERVE_DATA_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Swerve Data", TelemetryConstants.SHOULD_SWERVE_DATA_COMMUNICATE);
      TelemetryConstants.SHOULD_SWERVE_CTRE_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Swerve CTRE", TelemetryConstants.SHOULD_SWERVE_CTRE_COMMUNICATE);
      TelemetryConstants.SHOULD_THEMACHINE_DATA_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/TheMachine Data", TelemetryConstants.SHOULD_THEMACHINE_DATA_COMMUNICATE);
      TelemetryConstants.SHOULD_THEMACHINE_SIM_POSES_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/TheMachine Sim Poses", TelemetryConstants.SHOULD_THEMACHINE_SIM_POSES_COMMUNICATE);
      TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE = SmartDashboard.getBoolean("Telemetry/Scheduler", TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE);  
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public void periodic() {

    if(DriverStation.isEnabled())
    {
      m_theMachine.periodic();
    }

    if (TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE) {
        // Telemetry for the Command Scheduler
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
     }

    //updateTelemetrySettings();

    }
}
