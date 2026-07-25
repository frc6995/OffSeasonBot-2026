# Bug Report: IntakeIOSimTalonFX.java

**File:** `subsystems/intake/IntakeIOSimTalonFX.java`
**Severity:** 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🟡 BUG 1 — `kExtensionMOI` declared but never used

**Line:** 17

### Problem
```java
public static final double kExtensionMOI = 0.07;  // ← never used
```

The extension sim uses `ElevatorSim` with `kExtensionCarriageMassKg = 2.0` (line 18), not `kExtensionMOI`. The MOI field is dead code.

### Fix
Remove the unused field:
```java
// Delete: public static final double kExtensionMOI = 0.07;
```

---

## 🟡 BUG 2 — `kExtensionDrumRadiusMeters` defined locally but should use the canonical constant

**Line:** 19

### Problem
```java
private static final double kExtensionDrumRadiusMeters = 0.019;
```

If `IntakeConstants.kDrumCircumferenceMeters` is derived from a different radius, the sim and real code will disagree. Verify that `2 * Math.PI * kExtensionDrumRadiusMeters == IntakeConstants.kDrumCircumferenceMeters`.

### Fix
Either derive the circumference from the radius:
```java
// In IntakeConstants:
public static final double kExtensionDrumRadiusMeters = 0.019;
public static final double kDrumCircumferenceMeters = 2 * Math.PI * kExtensionDrumRadiusMeters;
```

Or verify the two values are consistent.

---

## 🟡 BUG 3 — No extension position feedback in `IntakeInputs` (sim writes Phoenix state but never reads it back)

### Problem

`IntakeIO.IntakeInputs` (from the interface) has voltage and current fields but **NO `extensionPositionMeters` field**. The sim (`IntakeIOSimTalonFX.java:91, 97`) writes the extension position into the Phoenix sim state via `setRawRotorPosition`, but the subsystem can never read the measured position back from `inputs`.

This means:
- The Superstructure can never ask "is the intake fully deployed?"
- The extension is open-loop from the subsystem's perspective
- Only Phoenix's internal closed-loop (MotionMagic) controls it, but the robot code has no observability

### Fix
Add a position field to the inputs:
```java
// In IntakeIO.IntakeInputs:
public double extensionPositionMeters;

// In IntakeIOTalonFX.updateInputs():
inputs.extensionPositionMeters = mechanismRotationsToMeters(
    m_extensionLeadMotor.getPosition().getValueAsDouble()
);

// In IntakeIOSimTalonFX.updateInputs():
inputs.extensionPositionMeters = extensionSim.getPositionMeters();
```
