# Bug Report: Hood.java

**File:** `subsystems/hood/Hood.java`
**Severity:** 🟠 HIGH + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🟠 BUG 1 — All real PID/feedforward gains are zero — hood will sag under gravity

**Lines:** 21–31 (in `HoodConstants`)

### Problem
```java
public static final double kP = 0;
public static final double kD = 0;
public static final double kS = 0;
public static final double kV = 0;
public static final double kG = 0;  // kG = 0 — gravity feedforward is zero!
```

The hood is an angular mechanism that holds against gravity. With **all gains at zero**, `MotionMagicVoltage` outputs zero volts the moment it reaches the target angle. The hood will immediately sag under gravity — no holding torque exists.

- `kG` (gravity feedforward) must be nonzero to hold the hood at an angle against gravity
- `kP` must be nonzero for any closed-loop control to function
- The comments in the code suggest these are "Tune PID/FF constants" TODOs

### Fix
```java
public static final double kP = 1.0;   // Start here, tune up
public static final double kD = 0.0;
public static final double kS = 0.0;
public static final double kV = 0.0;
public static final double kG = 0.2;    // Volts to hold at level — characterize with SysId
```

Run SysId on the hood mechanism to characterize kG, kS, kV, kP accurately.

---

## 🟡 BUG 2 — `applyLimits` clamps but never writes back the clamped value

**Lines:** In `Hood.java` (the `applyLimits` method)

### Problem
The subsystem reads `requestedAngle` (the raw input), calls `applyLimits` which clamps it, but the clamped value is never written back to `requestedAngle`. This means:
- If the operator requests an out-of-range angle, it gets clamped
- But `requestedAngle` stays at the out-of-range value
- Next cycle, the subsystem tries the bad value again
- The DS warning spam repeats every 20ms

### Fix
```java
// After clamping:
if (clamped != requestedAngle) {
    DriverStation.reportWarning("Hood angle clamped from %f to %f"
        .formatted(requestedAngle, clamped), false);
    requestedAngle = clamped;  // Write back the clamped value
}
```

Or accept the repeated clamping as a design choice (current behavior — at least it won't command out of range).
