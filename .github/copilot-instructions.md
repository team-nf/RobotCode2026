## Quick orientation for code-writing agents

This repository is a Java (Gradle + GradleRIO) WPILib command-based robot project. Key entry points and patterns you should know before editing: `Robot`, `RobotContainer`, and the `Subsystems/` packages.

- Entry points
  - `src/main/java/frc/robot/Robot.java` — TimedRobot subclass; runs the CommandScheduler and calls `RobotContainer.periodic()`.
  - `src/main/java/frc/robot/RobotContainer.java` — central wiring: controller bindings, subsystem construction, and `getAutonomousCommand()`.

- Subsystem layout and conventions (follow these precisely)
  - Location: `src/main/java/frc/robot/Subsystems/<Name>/`
  - Each subsystem typically has subpackages: `Hardware/`, `StateActions/`, `StateRequests/`, `Utils/`.
    - Example: `Subsystems/Shooter/Hardware/ShooterRealHardware.java` and `ShooterSimHardware.java`.
  - Hardware selection pattern: in subsystem constructor use simulation flag to pick hardware implementation: `if (Utils.isSimulation()) { new XSim(); } else { new XReal(); }`.
  - State machine pattern: subsystem holds a control-data object (e.g., `ShooterControlData`) and a `stateMachine()` method that schedules `StateActions` via `CommandScheduler` depending on `...ControlState` enums.
  - Requests/APIs: subsystems expose convenience methods that return Commands (e.g., `prepareRequest()`, `zeroRequest()`, `waitForShooterToBeReady()`) — these are used by `RobotContainer` to compose behaviors.

- Controller & command composition
  - `RobotContainer` uses `CommandXboxController` and binds buttons with `onTrue`/`onFalse`. Chains use `andThen(...)` and `WaitUntilCommand` for sequencing.
  - Example pattern from this repo:

    m_driverController.a()
        .onTrue(m_shooterSubsystem.prepareRequest()
        .andThen(m_shooterSubsystem.waitForShooterToBeReady()
        .andThen(m_feederSubsystem.feedRequest())));

- Telemetry & toggles
  - Telemetry is gated by `frc.robot.Constants.TelemetryConstants`. When toggles are enabled, code places objects on SmartDashboard using `SmartDashboard.putData(...)`.

- Build / run / test (safe, tested commands)
  - This project uses the Gradle wrapper (GradleRIO). On Windows PowerShell use the wrapper script in the repo root.
    - Build: `.\\gradlew.bat build`
    - Run tests: `.\\gradlew.bat test`
    - Deploy to RoboRIO: `.\\gradlew.bat deploy` (requires team number / robot connection)
  - The GradleRIO `wpi.sim` plugin is enabled (desktop simulation support). Use GradleRIO simulation tasks or the WPILib VS Code extension for simulation — prefer Gradle tasks where possible.

- Files & constants to consult when changing behavior
  - `Constants/` — robot-wide numeric and boolean flags (drive ports, telemetry toggles, states thresholds).
  - `Subsystems/*/Hardware/*` — implement both Real and Sim hardware here; add hardware update calls consistent with existing subsystems.
  - `Subsystems/*/StateActions` and `StateRequests` — prefer adding new actions/requests here rather than inlining command logic in `RobotContainer`.

- Style and safety notes (do not invent unobservable behavior)
  - Preserve the existing command-based architecture: add new Requests/Actions that return Commands rather than scheduling imperative loops directly.
  - Respect simulation vs real hardware switches. If you add new hardware, implement a Sim and Real variant and select using `Utils.isSimulation()`.
  - Avoid changing scheduler behavior in `Robot.java`; other code relies on `CommandScheduler.getInstance().run()` being called every `robotPeriodic()`.

If any section is unclear or you'd like more examples (e.g., how to add a new subsystem, or a minimal StateAction implementation), tell me which piece to expand and I will iterate.
