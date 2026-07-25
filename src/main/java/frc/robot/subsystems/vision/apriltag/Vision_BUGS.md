# Bug Report: Vision Subsystem (AprilTagVision, RealATVision, NoneATVision, AprilTagModule)

**Files:**
- `subsystems/vision/apriltag/AprilTagVision.java`
- `subsystems/vision/apriltag/RealATVision.java`
- `subsystems/vision/apriltag/NoneATVision.java`
- `subsystems/vision/apriltag/AprilTagModule.java`

**Severity:** 🔴 CRITICAL + 🟠 HIGH
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Entire vision subsystem is never instantiated (fully unwired)

### Problem

`RealATVision`, `AprilTagVision`, `NoneATVision`, and `AprilTagModule` are **never instantiated** anywhere in the project:

- `RobotContainer` does not create a vision instance
- No `addVisionMeasurement` plumbing exists in `Superstructure` or `CommandSwerveDrivetrain`'s periodic
- No `RealATVision.periodic()` is ever scheduled

The swerve drivetrain provides `addVisionMeasurement(Pose2d, double, Matrix)` in `CommandSwerveDrivetrain.java:309`, but it is **never called**. The robot has no AprilTag-based pose correction. Odometry drift will accumulate unbounded throughout a match.

### Fix

In `RobotContainer.java`:
```java
private final RealATVision vision;

public RobotContainer() {
    ...
    // Instantiate vision with gyro rotation supplier and pose reset consumer
    vision = new RealATVision(
        () -> new Rotation3d(m_drivetrain.getState().Pose.getRotation()),
        m_drivetrain::resetPose  // or a custom pose reset method
    );
    
    // Register vision periodic
    // Either make it a Subsystem or schedule it:
    new Command() {
        @Override
        public void execute() { vision.periodic(); }
    }.schedule();  // Or better: register as a Subsystem
}
```

---

## 🔴 BUG 2 — `resetPose` Consumer accepted but never called (pose never seeded from vision)

**File:** `RealATVision.java:58, 67–69`

### Problem
```java
private final Consumer<Pose2d> resetPose;           // line 58

public RealATVision(Supplier<Rotation3d> gyroRotation, Consumer<Pose2d> resetPose) {
    this.gyroRotation = gyroRotation;
    this.resetPose = resetPose;                      // line 69 — stored...
    // ... but resetPose is NEVER CALLED anywhere in the class
}
```

The constructor accepts a `Consumer<Pose2d>` for resetting the robot's pose, but **`resetPose.accept(...)` is never invoked anywhere in the class**. Even the gyro-seeding branch (line 84–93) only sets `headingSeeded = true` and publishes to NetworkTables — but never actually resets the robot's pose to the MegaTag1 estimate.

Even if someone wires up vision (fixing BUG 1), the gyro/pose will never be seeded from AprilTag data.

### Fix

In the seeding branch (lines 84–93):
```java
if(DriverStation.isDisabled() || !headingSeeded) {
    for(AprilTagModule limelight : limelights) {
        limelight.periodic();
        var result = limelight.getPose(false);  // MegaTag1
        if(result.isPresent()
            && result.get().tagCount() > 0  // Only seed if we have tags
            && result.get().estimatedPose().getTranslation().getDistance(Translation2d.kZero) > 0.05) {
            
            // ACTUALLY RESET THE POSE:
            resetPose.accept(result.get().estimatedPose());
            
            // Also pass to drivetrain's vision estimator:
            // (this should be done in the Superstructure periodic or a separate method)
            
            estimates.add(result.get());
            headingSeeded = true;
        }
    }
    seededPosePublisher.accept(new Pose3d(Translation3d.kZero, gyroRotation.get()));
}
```

---

## 🔴 BUG 3 — NaN can enter the pose estimator when tagCount = 0

**File:** `subsystems/vision/apriltag/AprilTagModule.java:174–180`

### Problem
```java
for(int i = 0; i < tagCount; i++) {
    int baseIndex = 11 + (i * valsPerFiducial);
    avgAmbiguity += poseArray[baseIndex + 6];
}
avgAmbiguity /= tagCount;        // line 180 — DIVISION BY ZERO if tagCount == 0
```

When `tagCount == 0`, `avgAmbiguity = 0.0 / 0 = NaN`. This NaN propagates through:
1. The `AprilTagEstimate` record wraps `NaN` into `avgAmbiguity`
2. `AprilTagVision.getStdDevsMT1` computes `xydevs = k * dist² / tagCount²` → also NaN
3. `addVisionMeasurement(pose, timestamp, NaN_stdDevs)` — a NaN std-dev **corrupts the Kalman filter** and can wipe the entire pose estimate

The `poseArray.length == 11` path (no fiducials) can reach this line because `expectedTotalVals == 11 + 7*0 == 11` matches.

### Fix
```java
// Guard the division:
if (tagCount > 0) {
    avgAmbiguity /= tagCount;
} else {
    // Skip emitting an estimate when there are no tags
    return Optional.empty();
}
```

Also add a `tagCount > 0` check before calling `getStdDevsMT1` / `getStdDevsMT2` in `AprilTagVision`:
```java
// In getStdDevsMT1:
if (estimate.tagCount() == 0) {
    return new double[] { Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE };
}
```

---

## 🟠 BUG 4 — Dead code in `RealATVision.periodic()`: impossible `if(!headingSeeded)` check

**File:** `RealATVision.java:84–95`

### Problem
```java
if(DriverStation.isDisabled() || !headingSeeded) {
    // seeding branch...
} else {
    if(!headingSeeded) headingSeeded = true;   // line 95 — DEAD CODE
    ...
}
```

The `else` branch executes when `!isDisabled() && headingSeeded` (because the `if` condition is `isDisabled() || !headingSeeded`). So inside `else`, `headingSeeded` is **always true**. The `if(!headingSeeded) headingSeeded = true;` can never fire — it's dead code.

### Fix
Remove line 95:
```java
} else {
    // headingSeeded is guaranteed true here — no check needed
    for(AprilTagModule limelight : limelights) {
        limelight.periodic();
        limelight.seedOrientation(gyroRotation.get());
        var estSupp = limelight.getPose();
        if(estSupp.isPresent()) {
            estimates.add(estSupp.get());
        }
    }
}
```

---

## 🟠 BUG 5 — Gyro seeding uses MegaTag1 without verifying tag quality

**File:** `RealATVision.java:88`

### Problem
```java
var result = limelight.getPose(false);  // false = MegaTag1
if(result.isPresent() && result.get().estimatedPose().getTranslation().getDistance(Translation2d.kZero) > 0.05) {
    estimates.add(result.get());
    headingSeeded = true;
}
```

The code seeds the gyro from MegaTag1 when:
- A pose is present
- The pose is more than 5cm from the origin

But it does NOT check:
- `tagCount > 0` (tags must be visible)
- `avgAmbiguity < threshold` (ambiguous poses are unreliable)
- `tagDistance < maxDistance` (far tags are inaccurate)

A single far tag with high ambiguity could seed the gyro to garbage.

### Fix
```java
var result = limelight.getPose(false);
if(result.isPresent()
    && result.get().tagCount() > 0
    && result.get().avgAmbiguity() < 0.5  // Lower is better
    && result.get().estimatedPose().getTranslation().getDistance(Translation2d.kZero) > 0.05) {
    
    resetPose.accept(result.get().estimatedPose());
    estimates.add(result.get());
    headingSeeded = true;
}
```

---

## 🟠 BUG 6 — `NoneATVision` constructor signature doesn't match `RealATVision`

**File:** `NoneATVision.java:13`

### Problem
```java
// RealATVision:
public RealATVision(Supplier<Rotation3d> gyroRotation, Consumer<Pose2d> resetPose)

// NoneATVision:
public NoneATVision(Supplier<Rotation3d> gyroRotation, Consumer<Rotation3d> resetRotation)
```

`RealATVision` takes `Consumer<Pose2d>` (full pose). `NoneATVision` takes `Consumer<Rotation3d>` (just rotation). The signatures don't match — if someone substitutes one for the other, the caller must pass different parameters. This breaks the abstraction pattern.

### Fix
```java
// Match the RealATVision signature:
public NoneATVision(Supplier<Rotation3d> gyroRotation, Consumer<Pose2d> resetPose) {}
```

---

## 🟡 BUG 7 — Connection warning prints at wrong rate

**File:** `subsystems/vision/apriltag/AprilTagModule.java:76`

### Problem
```java
if(((int)Timer.getFPGATimestamp()) % 3.0 == 0 && !isConnected()) {
    DriverStation.reportError(...);
}
```

`(int)Timer.getFPGATimestamp()` truncates to integer seconds. The warning fires only during the one-second window where `seconds % 3 == 0`, producing ~20 duplicate errors per "active" second, then 2 seconds of silence. This isn't a proper debounce.

### Fix
Use a debounced trigger:
```java
private boolean wasConnected = true;
...
boolean connected = isConnected();
if (!connected && wasConnected) {
    DriverStation.reportError("Limelight " + limelightID + " not connected!", false);
}
wasConnected = connected;
```

---

## 🟡 BUG 8 — `if(true)` leftover debug in `AprilTagModule.updateTelemetry`

**File:** `AprilTagModule.java:90`

### Problem
An `if(true)` conditional is a leftover debug artifact. It should be removed.

### Fix
Remove the `if(true)` wrapper entirely.
