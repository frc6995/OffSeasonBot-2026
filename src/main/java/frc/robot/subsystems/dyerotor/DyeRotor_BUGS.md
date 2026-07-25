# Bug Report: DyeRotor.java

**File:** `subsystems/dyerotor/DyeRotor.java`
**Severity:** 🔴 CRITICAL + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Indexer RPM passed to a voltage request (units mismatch)

**Lines:** 109, and `DyeRotorIOTalonFX.java:121`

### Problem

In `DyeRotor.java` (the subsystem):
```java
case SPIN -> io.setIndexVoltage(resolveIndexTargetRPM(indexState));  // line 109
```

`resolveIndexTargetRPM()` returns a value in **RPM** (±6 RPM or 0 depending on state). But `setIndexVoltage(double)` in `DyeRotorIOTalonFX` expects **volts**:
```java
public void setIndexVoltage(double volts) {
    m_indexerLead.setControl(m_indexerRequest.withOutput(volts));  // m_indexerRequest is a VoltageOut
}
```

So when the indexer should be "active," the code passes **6 volts** to the indexer motor (interpreting 6 RPM as 6 volts), not 6 RPM. The indexer will spin at whatever speed 6V produces — unregulated, no closed-loop, no PID. This is a critical units mismatch.

### Root Cause
The method is named `setIndexVoltage` (accepting volts) but the caller passes RPM. Either:
- The IO method should be `setIndexVelocityRPM(double rpm)` using a `VelocityVoltage` request (the symmetric of `setSpinVelocity`), OR
- `resolveIndexTargetRPM` is misnamed and should return volts.

Given that `kIndexKP`, `kIndexKS`, `kIndexKV` are all 0 (never tuned), the indexer was never properly finished.

### Fix — Option A (change IO to velocity control)
```java
// In DyeRotorIO interface:
public void setIndexVelocityRPM(double rpm);

// In DyeRotorIOTalonFX:
private final VelocityVoltage m_indexerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

public void setIndexVelocityRPM(double rpm) {
    m_indexerLead.setControl(m_indexerVelocityRequest.withVelocity(rpm / 60.0)); // rotations per second
}

// In DyeRotor.java:
case SPIN -> io.setIndexVelocityRPM(resolveIndexTargetRPM(indexState));
```

### Fix — Option B (change caller to volts)
```java
// In DyeRotor.java:
case SPIN -> io.setIndexVoltage(resolveIndexTargetVolts(indexState)); // rename method, return volts
```

Option A is recommended for consistency with `setSpinVelocity` which already uses velocity control.

---

## 🟡 BUG 2 — Indexer PID gains are all zero

**Lines:** In `DyeRotorIOTalonFX.java` config or `DyeRotor.DyeRotorConstants`

### Problem
`kIndexKP = 0`, `kIndexKS = 0`, `kIndexKV = 0` — the indexer's Slot0 gains are all zero. Even after fixing the units mismatch (BUG 1), a `VelocityVoltage` request with P=0, KV=0, KS=0 will produce zero output — the motor won't move.

### Fix
Tune the indexer PID/FF gains:
```java
config.Slot0 = new Slot0Configs()
    .withKP(kIndexKP)    // Start with ~0.1
    .withKS(kIndexKS)    // Start with ~0.05 (static friction)
    .withKV(kIndexKV);   // Start with ~0.12 (voltage per RPS)
```

---

## 🟡 BUG 3 — CAN IDs collide with Intake (but on different CAN buses)

**Lines:** In `DyeRotorConstants`

### Problem
- `DyeRotor.kSpinMotorCANID = 30` — same as `Intake.kROLLER_LEAD_MOTOR_ID = 30`
- `DyeRotor.kLeadIndexMotorCANID = 31` — same as `Intake.kROLLER_FOLLOWER_MOTOR_ID = 31`
- `DyeRotor.kFollowIndexMotorCANID = 32` — same as `Intake.kEXTENSION_LEAD_MOTOR_ID = 32`

These are on **different CAN buses** (DyeRotor = LowerBus, Intake = UpperBus), so the IDs can be reused without conflict. However, this is confusing for debugging — anyone seeing "device 30" on the CAN bus won't know which motor it is without checking the bus name.

### Fix
Add clear documentation or partition the IDs:
```java
// DyeRotor uses CAN IDs 30-33 on LowerBus
// Intake uses CAN IDs 30-34 on UpperBus
// IDs can be reused because they're on different physical buses
```

Or use non-overlapping ID ranges across all subsystems for clarity, even across buses.
