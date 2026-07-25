# Bug Report: Superstructure.java

**File:** `subsystems/Superstructure.java`
**Severity:** 🟠 HIGH + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🟠 BUG 1 — `requestXxx` methods don't declare subsystem requirements

**Lines:** 62–104

### Problem
```java
public Command requestRobotScoring() {
    return Commands.runOnce(() -> {
        robotState = RobotState.SCORING;
        m_dyeRotor.setState(DyeRotorState.SPIN);
        m_turret.setState(TurretState.ACTIVE);
        m_flywheel.setState(FlywheelState.ACTIVE);
    });
}
```

`Commands.runOnce(...)` creates a command with **no subsystem requirements**. The CommandScheduler does not know this command touches `m_dyeRotor`, `m_turret`, `m_flywheel`, etc. A second conflicting command can be scheduled simultaneously and overwrite the state mid-sequence. The scheduler has no way to prevent concurrent access.

### Fix
```java
public Command requestRobotScoring() {
    return Commands.runOnce(() -> {
        robotState = RobotState.SCORING;
        m_dyeRotor.setState(DyeRotorState.SPIN);
        m_turret.setState(TurretState.ACTIVE);
        m_flywheel.setState(FlywheelState.ACTIVE);
    }, m_dyeRotor, m_turret, m_flywheel, m_hood, m_intake);  // Declare requirements
}
```

Or use a custom Command class with `addRequirements(...)`.

---

## 🟡 BUG 2 — Subsystem fields are `public` mutable

**Lines:** 36–40

### Problem
```java
public Intake m_intake;
public Hood m_hood;
public Flywheel m_flywheel;
public Turret m_turret;
public DyeRotor m_dyeRotor;
```

Any code can do `m_superStructure.m_flywheel = null;` or replace with another instance mid-match. This is a serious encapsulation failure.

### Fix
Make them `private final`:
```java
private final Intake m_intake;
private final Hood m_hood;
private final Flywheel m_flywheel;
private final Turret m_turret;
private final DyeRotor m_dyeRotor;
```

If other classes need access, provide getters:
```java
public Intake getIntake() { return m_intake; }
public Hood getHood() { return m_hood; }
// etc.
```

---

## 🟡 BUG 3 — `runOnce` fires only once, but subsystems need continued periodic commanding

**Lines:** 62–104

### Problem

`Commands.runOnce(...)` executes exactly one time. It sets the state on each subsystem, then the subsystems' own `periodic()` methods pick up the new state each cycle. This works because the state machine is retained in the subsystems. **However**, there's no explicit `end()` action — if the command is cancelled, the subsystems remain in their last state with no cleanup. For example, if `requestRobotScoring` is cancelled, the turret stays `ACTIVE` and the flywheel keeps spinning.

### Fix
Use `StartEndCommand` or add an `end()` handler:
```java
public Command requestRobotScoring() {
    return Commands.runOnce(() -> {
        robotState = RobotState.SCORING;
        m_dyeRotor.setState(DyeRotorState.SPIN);
        m_turret.setState(TurretState.ACTIVE);
        m_flywheel.setState(FlywheelState.ACTIVE);
    }, m_dyeRotor, m_turret, m_flywheel)
    .andThen(Commands.idle(m_dyeRotor, m_turret, m_flywheel));
    // The idle keeps them as requirements until the command is cancelled
}
```

Or use `StartEndCommand`:
```java
return new StartEndCommand(
    () -> { /* set SCORING state */ },
    () -> { /* reset to IDLE state */ },
    m_dyeRotor, m_turret, m_flywheel
);
```
