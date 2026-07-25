# Bug Report: AutoAlign.java & AutoAlignFixedHeading.java

**Files:**
- `util/AutoAlign.java`
- `util/AutoAlignFixedHeading.java`

**Severity:** 🔴 CRITICAL + 🟠 HIGH
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Robot-relative velocities passed to field-centric swerve request

**File:** `util/AutoAlign.java:211–216`
**Severity:** CRITICAL — robot drives in wrong direction when rotated

### Problem
```java
APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

m_drivetrain.setControl(m_request                        // SwerveRequest.FieldCentricFacingAngle
    .withVelocityX(out.vx())                              // ← Autopilot outputs ROBOT-RELATIVE vx
    .withVelocityY(out.vy())                              // ← ROBOT-RELATIVE vy
    .withTargetDirection(out.targetAngle()));             // ← heading target
```

**API verification (from Autopilot 1.6.1 jar inspection):**
- `Autopilot.calculate(Pose2d, ChassisSpeeds, APTarget)` — takes `ChassisSpeeds` which is robot-relative (per WPILib convention)
- `APResult.vx()` / `vy()` returns `LinearVelocity` — **robot-relative** velocities
- `CommandSwerveDrivetrain.getChassisSpeeds()` returns `state().Speeds` which is robot-relative in Phoenix 6

`m_request` is a `SwerveRequest.FieldCentricFacingAngle` (line 72–75), which expects **field-centric** velocities (X = forward in the blue alliance frame, Y = left in the blue alliance frame).

When the robot is rotated to any heading other than 0° (facing blue's +X), the robot-relative velocities are projected onto the wrong axes — the robot **drives in the wrong direction**. For example, if the robot is facing 90° (toward blue's +Y) and Autopilot says "go forward at 1 m/s" (robot-relative vx=1), the `FieldCentricFacingAngle` interprets that as "move 1 m/s in the field's X direction" — but the robot's forward is field Y. Result: the robot strafes sideways instead of going forward.

### Fix

**Option A** — Convert Autopilot's robot-relative output to field-relative:
```java
APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

// Convert robot-relative to field-relative
Rotation2d heading = swerveState.Pose.getRotation();
Translation2d fieldVel = new Translation2d(out.vx().in(MetersPerSecond), out.vy().in(MetersPerSecond))
    .rotateBy(heading);

m_drivetrain.setControl(m_request
    .withVelocityX(fieldVel.getX())
    .withVelocityY(fieldVel.getY())
    .withTargetDirection(out.targetAngle()));
```

**Option B** — Use `SwerveRequest.RobotCentric` with a facing angle variant (if available):
```java
// Use RobotCentric and let Autopilot own both translation and rotation
SwerveRequest.RobotCentricFacingAngle robotRequest = new SwerveRequest.RobotCentricFacingAngle()
    .withVelocityX(out.vx())
    .withVelocityY(out.vy())
    .withTargetDirection(out.targetAngle());
```

---

## 🔴 BUG 2 — Two competing heading controllers cause drift/circling

**File:** `util/AutoAlign.java:72–75, 211–216` and `util/AutoAlignFixedHeading.java:88–105`
**Severity:** CRITICAL — trajectory planning is inconsistent with execution

### Problem

Autopilot plans a **holonomic trajectory** — it decides both the translation path and the chassis heading rotation plan. The output includes `vx`, `vy`, and `targetAngle`.

But `SwerveRequest.FieldCentricFacingAngle` has its **own internal heading PID** (configured at line 75: `withHeadingPID(5, 0, 0)`). This PID controller independently forces the chassis to face `targetAngle` via a simple P controller.

**Two problems arise:**

1. **Autopilot's translation assumes the chassis follows Autopilot's planned heading.** But the actual chassis heading is controlled by the `FieldCentricFacingAngle`'s P controller, which may lag or overshoot. The translation vector is planned for one heading but executed at a different one — the robot drifts off the path.

2. **`FieldCentricFacingAngle` discards Autopilot's planned angular velocity (`omega`)** and replaces it with its own P-controlled heading. Autopilot's smooth trajectory heading plan is thrown away.

### Fix

**Option A** — Let Autopilot own everything (use `FieldCentric` not `FacingAngle`):
```java
m_request = new SwerveRequest.FieldCentric()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
// In execute():
m_drivetrain.setControl(m_request
    .withVelocityX(/* field-relative out.vx */)
    .withVelocityY(/* field-relative out.vy */)
    .withRotationalRate(/* out.omega() converted to field-frame */));
```

**Option B** — Use `FacingAngle` but feed Autopilot the ACTUAL heading, not let it plan one:
```java
// If Autopilot's targetAngle is just the final target heading (constant),
// then the FacingAngle P controller handling rotation is fine —
// but the translation must still account for the actual chassis heading.
```

---

## 🔴 BUG 3 — `DEFAULT_MAX_VELOCITY` exceeds physical maximum

**File:** `util/AutoAlign.java:28`, cross-ref `TunerConstants.java:85`

### Problem
```java
public static double DEFAULT_MAX_VELOCITY = 5.5;  // comment says "physical max is 5.5 m/s^2"
```

But `TunerConstants.java:85` shows `kSpeedAt12Volts = 4.39 m/s`. The comment also has wrong units (`m/s²` is acceleration, `m/s` is velocity).

Since `DEFAULT_MAX_VELOCITY = 5.5 m/s > kSpeedAt12Volts = 4.39 m/s`, Autopilot will command velocities the chassis **cannot physically reach**. The velocity profile will undershoot, the robot will lag behind the planned trajectory, and AutoAlign will overshoot or oscillate near the target.

### Fix
```java
// Match the physical maximum:
public static final double DEFAULT_MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
// = 4.39 m/s — or use 4.0 to leave headroom
```

---

## 🔴 BUG 4 — Hardcoded to Blue alliance perspective (breaks on Red)

**File:** `util/AutoAlign.java:73`

### Problem
```java
.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
```

The perspective is hardcoded to Blue. When the robot is on the Red alliance:
- `RobotContainer`'s teleop drive uses `setOperatorPerspectiveForward(kRedAlliancePerspectiveRotation)` (180°)
- But AutoAlign forces Blue perspective

If the caller passes a target pose in the red alliance frame (which is natural for red-side coordinates), the alignment will point 180° off. The robot drives to the mirror of the intended target.

`Autos.java:61` even has the comment: `// In actual use, this pose will need to be flipped` — confirming the alliance flip was never implemented.

### Fix
```java
ForwardPerspectiveValue perspective = DriverStation.getAlliance()
    .map(a -> a == Alliance.Red 
        ? ForwardPerspectiveValue.RedAlliance 
        : ForwardPerspectiveValue.BlueAlliance)
    .orElse(ForwardPerspectiveValue.BlueAlliance);

m_request = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(perspective)
    .withHeadingPID(5, 0, 0);  // ← Also needs tuning, see BUG 5
```

---

## 🟠 BUG 5 — Heading PID hardcoded `5, 0, 0` with no D term — oscillation risk

**File:** `util/AutoAlign.java:75`

### Problem
```java
.withHeadingPID(5, 0, 0)  // P=5, I=0, D=0 — "Replace with constants later"
```

A pure P controller (no derivative) on a swerve heading will **overshoot and oscillate** — especially when the translation motion also perturbs the heading. On a real 4.39 m/s swerve, P=5 may be too much (causing saturation and oscillation) or too little (unable to hold against translation disturbance during alignment).

### Fix
```java
// Move to constants and add D term:
.withHeadingPID(kHeadingP, kHeadingI, kHeadingD)
// Start with P=5, D=0.5 and tune
```

---

## 🟠 BUG 6 — `isFinished()` calls `getState()` a second time per cycle (stale/race)

**File:** `util/AutoAlign.java:228–230`

### Problem
```java
// In execute():
swerveState = m_drivetrain.getState();  // first read
APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

// In isFinished():
return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);  // second read!
```

`isFinished()` calls `getState()` again — a fresh odometry update may have arrived between `execute()` and `isFinished()`. The pose used to drive the robot (`execute()`) differs from the pose that decides whether to stop (`isFinished()`). The robot may stop while still moving (the `execute()` pose said "keep going" but the fresher `isFinished()` pose said "at target").

### Fix
```java
// Cache the pose from execute and reuse in isFinished:
private Pose2d lastPose;

// In execute():
swerveState = m_drivetrain.getState();
lastPose = swerveState.Pose;

// In isFinished():
return kAutopilot.atTarget(lastPose, m_target);
```

---

## 🔴 BUG 7 — `AutoAlignFixedHeading.m_realTarget` is null if `isFinished()` runs before `execute()`

**File:** `util/AutoAlignFixedHeading.java:109`

### Problem
```java
// Field initialized to null:
private APTarget m_realTarget;  // null

// Only set in execute():
public void execute() {
    ...
    m_realTarget = /* ... */;
}

// But isFinished() reads it:
public boolean isFinished() {
    return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_realTarget);  // NPE if null!
}
```

WPILib normally calls `execute()` before `isFinished()`, but `ParallelRaceGroup` and `.until()` decorations can interleave — potentially calling `isFinished()` before `execute()` on the first cycle.

### Fix
```java
// Initialize in constructor:
public AutoAlignFixedHeading(...) {
    ...
    m_realTarget = m_target;  // Safe default
}
```

---

## 🟠 BUG 8 — `toPoseUntilWithinDistance` bypasses APProfile tolerances

**File:** `util/AutoAlign.java:137–147`

### Problem
```java
public Command toPoseUntilWithinDistance(APTarget target, double distance) {
    return new AutoAlign(target, m_drivetrain)
        .until(() -> TriggerUtil.isWithinRadius(...distance...));
}
```

The `until()` decorator ends the command when within a radius. But `AutoAlign.isFinished()` also ends at `kAutopilot.atTarget()` (typically with a tighter tolerance). Since `until()` fires first (usually at a larger radius than the APProfile's ErrorXY=6cm), the robot stops **short of Autopilot's convergence point**. The APProfile tolerances are silently ignored.

### Fix
Document this behavior or remove the radius option:
```java
/**
 * Note: This ends the command at the specified radius, which may be LARGER
 * than the APProfile's ErrorXY tolerance. The robot will stop short of
 * Autopilot's convergence point.
 */
public Command toPoseUntilWithinDistance(APTarget target, double distance) {
    ...
}
```

---

## 🟡 BUG 9 — Static `APProfile` fields are mutable (non-final)

**File:** `util/AutoAlign.java:43–64`

### Problem
```java
public static APProfile kDefaultProfile = ...;     // not final
public static APProfile kSlowDriveProfile = ...;    // not final
```

Any code can do `AutoAlign.kDefaultProfile = null;` and break all future auto-aligns.

### Fix
Make all static profile fields `final`:
```java
public static final APProfile kDefaultProfile = ...;
```
