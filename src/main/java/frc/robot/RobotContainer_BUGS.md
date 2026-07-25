# Bug Report: RobotContainer.java & Robot.java

**Files:**
- `RobotContainer.java`
- `Robot.java`

**Severity:** 🟠 HIGH + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🟠 BUG 1 — `Autos` is dead code (never instantiated)

**File:** `RobotContainer.java`

### Problem

`Autos.java` fully implements `AutoChooser` + BLine path following + AutoAlign integration. But:
- `RobotContainer` never creates an `Autos` instance
- `Robot.java:60` calls `m_robotContainer.getAutonomousCommand()` which returns a **hardcoded "drive forward 5 seconds"** routine
- The complex auto path system is completely unused

### Fix
```java
// In RobotContainer:
private final Autos autos;

public RobotContainer() {
    ...
    autos = new Autos(m_drivetrain);
    SmartDashboard.putData("Auto", autos.getAutoChooser());
    configureBindings();
}

public Command getAutonomousCommand() {
    return autos.selectedCommand();
    // Or: return autos.getAutoChooser().getSelected();
}
```

---

## 🟠 BUG 2 — `RobotVisualizer` is dead code

### Problem

`RobotVisualizer.java` is never instantiated. `addShooterPivot`, `addHood`, `addIntake`, `addDyeRotor` and `setupVisualizer` are never called. None of the subsystems construct `MechanismLigament2d`s to attach.

### Fix
Either:
- Complete the visualizer: hook up each subsystem's `MechanismLigament2d` in their `periodic()` methods and call `RobotVisualizer.setupVisualizer()` from `RobotContainer`
- OR delete the file until ready to use it

---

## 🟠 BUG 3 — `autonomousInit` sim-exit command is fire-and-forget

**File:** `Robot.java:50–59`

### Problem
```java
if (RobotBase.isSimulation()) {
    Commands.waitSeconds(autoSimTime)
        .andThen(() -> { DriverStationSim.setEnabled(false); ... })
        .onlyWhile(DriverStation::isAutonomousEnabled)
        .schedule();    // fire-and-forget — not tracked
}
m_autonomousCommand = m_robotContainer.getAutonomousCommand();
```

The `.schedule()` call creates an anonymous command that is never stored or cancelled. `teleopInit` (line 75–77) only cancels `m_autonomousCommand`, not this sim-exit command. If the user toggles modes in sim, a stray "disable in 20s" command may still be pending.

### Fix
Track the command and cancel it:
```java
private Command m_simExitCommand;

// In autonomousInit:
if (RobotBase.isSimulation()) {
    m_simExitCommand = Commands.waitSeconds(autoSimTime)
        .andThen(() -> { DriverStationSim.setEnabled(false); ... })
        .onlyWhile(DriverStation::isAutonomousEnabled);
    m_simExitCommand.schedule();
}

// In teleopInit:
if (m_simExitCommand != null) m_simExitCommand.cancel();
if (m_autonomousCommand != null) m_autonomousCommand.cancel();
```

---

## 🟡 BUG 4 — Unused imports of sim IO classes

**File:** `RobotContainer.java:24, 26, 28, 30, 33`

### Problem
Imports `DyeRotorIOSimTalonFX`, `FlywheelIOSimTalonFX`, `HoodIOSimTalonFX`, `IntakeIOSimTalonFX`, `TurretIOSimTalonFX` — none are referenced. The sim/real switch happens inside `Superstructure`'s constructor.

### Fix
Delete the unused imports.

---

## 🟡 BUG 5 — `MaxSpeed` / `MaxAngularRate` are non-static non-final

**File:** `RobotContainer.java:37–38`

### Problem
```java
private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
```

These are instance fields (recreated on every `RobotContainer` construction, e.g. on robot restart) and mutable.

### Fix
```java
private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
```

---

## 🟡 BUG 6 — `POI.TEST_POSE` has unresolved alliance flip

**File:** `util/POI.java:11`

### Problem
```java
public static Supplier<Pose2d> TEST_POSE = () -> {
    //Need to flip
    return new Pose2d(0,0,0);
};
```

The `//Need to flip` comment confirms the pose needs alliance flipping. If consumed on red alliance (e.g. by AutoAlign), the robot will drive to the wrong physical location (blue's origin).

### Fix
```java
public static Supplier<Pose2d> TEST_POSE = () -> {
    Pose2d base = new Pose2d(0, 0, Rotation2d.kZero);
    return DriverStation.getAlliance()
        .map(a -> a == Alliance.Red ? FlippingUtil.flip(base) : base)
        .orElse(base);
};
```
