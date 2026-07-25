# Bug Report: util/ Classes

**Files:**
- `util/UnitUtil.java`
- `util/TriggerCommand.java`
- `util/SwerveDriveStateLogger.java`
- `util/Telemetry.java`
- `util/LimelightHelpers.java`

**Severity:** 🟠 HIGH + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🟠 BUG 1 — `LimelightHelpers.getBotPoseEstimate` timestamp is not FPGA time

**File:** `util/LimelightHelpers.java:911–931`

### Problem
```java
adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
```

`timestamp` is the **NT server wall-clock** timestamp in microseconds. `latency` is in milliseconds. The arithmetic is correct for producing a "seconds" value, BUT:

WPILib's `addVisionMeasurement(pose, timestampSeconds)` expects `timestampSeconds` to be in **FPGA time** (i.e., `Timer.getFPGATimestamp()` domain). The NT server's wall clock is NOT synced to the RIO's FPGA timer. Mixing wall-clock timestamps with FPGA timestamps **breaks the Kalman filter's covariance propagation** — the pose estimator may reject valid measurements or accept stale ones.

This is the classic LimelightHelpers timestamp bug.

### Fix
Use the Limelight's `ts_us` (FPGA microsecond capture) field if available, or convert:
```java
// If using NT timestamps, convert to FPGA time:
double fpgaTimestamp = Timer.getFPGATimestamp() - (latency / 1000.0) - ( pipeline_latency );
// Or use Limelight's ts_nt if available and sync the NT server clock to the RIO
```

---

## 🟠 BUG 2 — `SwerveDriveStateLogger` no null checks on state fields

**File:** `util/SwerveDriveStateLogger.java:20–26`

### Problem
```java
object.Pose, object.ModuleTargets, object.ModuleStates, object.ModulePositions, object.Speeds
```

These `SwerveDriveState` fields can be **null** before the first odometry update (e.g., called during `disabledInit` before the first periodic tick). Accessing them without null checks throws NPE → Epilogue logs an error repeatedly.

### Fix
```java
@Override
public void log(SwerveDriveState object) {
    if (object == null) return;
    if (object.Pose != null) dataLogger.log("cha/Pose", object.Pose, Pose2d.struct);
    if (object.Speeds != null) dataLogger.log("cha/Speeds", object.Speeds, ChassisSpeeds.struct);
    if (object.ModuleStates != null) dataLogger.log("mod/ModuleStates", object.ModuleStates, SwerveModuleState.struct);
    if (object.ModuleTargets != null) dataLogger.log("mod/ModuleTargets", object.ModuleTargets, SwerveModuleState.struct);
    if (object.ModulePositions != null) dataLogger.log("mod/ModulePositions", object.ModulePositions, SwerveModulePosition.struct);
}
```

---

## 🟡 BUG 3 — `UnitUtil.CW_180` and `CCW_180` are mutable (non-final)

**File:** `util/UnitUtil.java:13–14`

### Problem
```java
public static Angle CW_180 = Degrees.of(-180);
public static Angle CCW_180 = Degrees.of(180);
```

`public static` but **not `final`** — any code can reassign these and break all downstream consumers.

### Fix
```java
public static final Angle CW_180 = Degrees.of(-180);
public static final Angle CCW_180 = Degrees.of(180);
```

---

## 🟡 BUG 4 — `UnitUtil.isWithinTolerance` doesn't handle angle wrapping

**File:** `util/UnitUtil.java:25–29`

### Problem
For `Angle` inputs, this compares `baseUnitMagnitude()` linearly. For angles near ±π (180°), the true angular difference across the wrap is ignored — e.g., value = -179°, target = 179° reports a 358° error instead of 2°.

### Fix
For `Angle` types, use `MathUtil.isNear`:
```java
if (unit instanceof Angle) {
    return MathUtil.isNear(
        targetSupplier.get().baseUnitMagnitude(),
        valueSupplier.get().baseUnitMagnitude(),
        toleranceSupplier.get().baseUnitMagnitude()
    );
}
// else do the linear check
```

---

## 🟡 BUG 5 — `TriggerCommand` EventLoop never cleared — memory leak

**File:** `util/TriggerCommand.java:14, 58–84`

### Problem
- `m_eventLoop` is never cleared/closed. Bindings accumulate across re-schedules.
- The duplicate-command guard only runs when `!interrupt` — when `interrupt=true`, no dedup check happens.
- This class is never used anywhere — it's dead code.

### Fix
Delete the file. If kept, clear the `EventLoop` in `end()`:
```java
@Override
public void end(boolean interrupted) {
    m_eventLoop.clear();
    m_commands.clear();
}
```

---

## 🟡 BUG 6 — `Telemetry.telemeterize` re-sets `.type` field every cycle

**File:** `util/Telemetry.java:38`

### Problem
```java
fieldTypePub.set("Field2d");  // called every 20ms
```

### Fix
```java
// In constructor:
fieldTypePub.setDefault("Field2d");
// Remove the per-cycle set
```

---

## 🟡 BUG 7 — `Telemetry` has no null guard on `state`

**File:** `util/Telemetry.java:31`

### Problem
```java
m_poseArray[0] = state.Pose.getX();  // NPE if state or state.Pose is null
```

### Fix
```java
public void telemeterize(SwerveDriveState state) {
    if (state == null || state.Pose == null) return;
    ...
}
```

---

## 🟡 BUG 8 — `LimelightHelpers.toPose3D` no null guard on input array

**File:** `util/LimelightHelpers.java:836`

### Problem
If Jackson deserializes a `null` botpose from a malformed JSON payload, `toPose3D(null)` will NPE.

### Fix
```java
public static Pose3d toPose3D(double[] inData) {
    if (inData == null || inData.length < 6) return new Pose3d();
    ...
}
```

---

## 🟡 BUG 9 — `LimelightHelpers.extractArrayEntry` returns 0 for missing data

**File:** `util/LimelightHelpers.java:903`

### Problem
Returns `0` for out-of-bounds indices. When a missing `distToCamera` reads as `0`, consumers treat a missing-data pose as "very close, high-confidence" — bad covariance weighting.

### Fix
Document loudly or return `NaN` to make the absence obvious, and have callers filter `NaN` values.
