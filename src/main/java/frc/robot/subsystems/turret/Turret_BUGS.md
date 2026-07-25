# Bug Report: Turret.java

**File:** `subsystems/turret/Turret.java`
**Severity:** 🔴 CRITICAL + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — `requestedAngle` is read but never written (turret always commands 0°)

**Lines:** 10, 69, 87–90

### Problem
```java
private double requestedAngle;          // line 10 — defaults to 0.0 forever
...
case ACTIVE -> io.setAngle(requestedAngle);   // line 69 — always 0
...
public double getRequestedAngle() { return requestedAngle; }   // line 87 — getter exists, NO SETTER
```

There is **no `setRequestedAngle()`, `setAngle()`, `setTarget()`, or any method that writes `requestedAngle`**. The field is initialized to `0.0` by default and is never modified. Whenever `turretState == ACTIVE`, the IO is commanded to `0°` — the turret physically points wherever it was at boot and never moves.

The Superstructure's `requestScoring()`/`requestPassing()` set the turret state to `ACTIVE` but never give it a target angle. The entire turret aiming system is non-functional.

### Fix
Add a setter method:
```java
public void setRequestedAngle(double angle) {
    requestedAngle = angle;
}
```

Then call it from the Superstructure or aiming logic:
```java
// In Superstructure or a command:
m_turret.setRequestedAngle(computeTurretTargetAngle());
m_turret.setState(TurretState.ACTIVE);
```

---

## 🟡 BUG 2 — Periodic updates inputs BEFORE commanding (stale reads)

**Lines:** 64–72

### Problem
```java
@Override
public void periodic() {
    switch (turretState) {
        case DISABLED -> io.disable();
        case ACTIVE -> io.setAngle(requestedAngle);   // commands first
    }
    io.updateInputs(inputs);                          // then reads inputs from PRE-command state
}
```

The IO is commanded **before** `updateInputs` reads back the motor state. This means `inputs` always lags one cycle — `getAngle()` returns the position from *before* the latest `setAngle` command. Other subsystems (Intake, Flywheel) typically do `updateInputs` first, then command.

### Fix
```java
@Override
public void periodic() {
    io.updateInputs(inputs);   // read current state FIRST
    
    switch (turretState) {
        case DISABLED -> io.disable();
        case ACTIVE -> io.setAngle(requestedAngle);
    }
}
```

---

## 🟡 BUG 3 — All real PID/feedforward gains are zero

**Lines:** 18–23 (in `TurretConstants`)

### Problem
```java
public static final double kP = 0;
public static final double kD = 0;
public static final double kS = 0;
public static final double kV = 0;
public static final double kG = 0;  // kG = 0 for a turret that may need to hold against gravity/spring
```

With all gains at zero, `MotionMagicVoltage` outputs zero volts the moment it thinks it has reached the target. The turret will not hold position against any disturbance. Even if the turret is balanced (no gravity load), `kP` must be nonzero for Motion Magic to function.

### Fix
Tune the PID gains:
```java
public static final double kP = 1.0;   // Start here, tune up
public static final double kD = 0.0;
public static final double kS = 0.0;
public static final double kV = 0.0;
public static final double kG = 0.0;    // Add if turret has gravity/spring load
```
