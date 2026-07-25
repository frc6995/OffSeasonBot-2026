# Bug Report: TunerConstants.java

**File:** `generated/TunerConstants.java`
**Severity:** 🟡 MEDIUM (verify against physical robot)
**Reviewed:** 2026-07-25

> ⚠️ This file is **Tuner-generated**. Do not edit values directly — re-run Tuner X to regenerate. The items below are verification warnings, not code bugs.

---

## 🟡 VERIFY 1 — `kWheelRadius = Inches.of(2)` — verify against physical wheels

**Line:** 93

### Problem
```java
public static final Distance kWheelRadius = Inches.of(2);
```

Hardcoded to 2″ wheel radius. If the robot has different wheels (e.g., 3.875″ Colson, 4″ pneumatic), odometry and AutoAlign will be **systematically off by the radius ratio**. This is the single most common source of "AutoAlign drifts to the wrong spot" bugs.

### Fix
Verify against the physical wheel:
- Measure the actual wheel diameter with calipers
- If different from 2″, re-run Tuner X with the correct wheel size and regenerate this file

---

## 🟡 VERIFY 2 — `kDriveGearRatio = 7.03125` — verify against physical gearbox

**Line:** 91

### Problem
```java
public static final double kDriveGearRatio = 7.03125;
```

Standard SDS Mk4i L2 ratio is 7.1634, Mk4 L3 is 6.12, etc. `7.03125` is unusual — verify it matches the actual gearbox installed. Wrong gear ratio → wrong odometry ↔ wrong AutoAlign.

### Fix
- Check the gearbox model (Mk4, Mk4i, Mk4i L2, etc.)
- Verify the ratio matches the manufacturer's spec for the installed gearset
- If wrong, re-run Tuner X and regenerate

---

## 🟡 VERIFY 3 — `kSpeedAt12Volts = 4.39 m/s` — should match AutoAlign max

**Line:** 85

### Problem
```java
public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.39);
```

This is the measured physical max speed. But `AutoAlign.DEFAULT_MAX_VELOCITY = 5.5` (in `AutoAlign.java:28`) exceeds this — Autopilot commands velocities the chassis cannot reach.

### Fix
Fix `AutoAlign.DEFAULT_MAX_VELOCITY` to match:
```java
public static final double DEFAULT_MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
// = 4.39 m/s — or use slightly less for headroom
```

---

## 🟡 VERIFY 4 — Steer gains and stator current limit

**Line:** 27

### Problem
```java
// Steer gains: kP=100, kD=0.5, kS=0.1, kV=2.49
// Stator current limit: 60A (line 71)
```

These are Tuner-default-ish gains, not tuned to the specific robot. 60A stator on an azimuth motor is high and may cause thermal issues.

### Fix
- Verify steer gains produce stable module response on the physical robot
- Consider lowering stator current limit to 30-40A for azimuth motors unless sustained high torque is needed

---

## 🟡 VERIFY 5 — `pigeonConfigs = null` — Pigeon calibration silently skipped

**Line:** 76

### Problem
```java
public static final Pigeon2Configuration pigeonConfigs = null;  // ← null
```

CTRE's `withPigeon2Configs(null)` silently skips applying Pigeon mount calibration. If the Pigeon is mounted at an angle or has a known offset, it won't be compensated.

### Fix
If the Pigeon has a mount offset, create a proper config:
```java
public static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration()
    // Add mount calibration if needed:
    // .withMountPose... 
    ;
```
Or verify the Pigeon is mounted flat with no rotation offset.
