# Bug Report: FlywheelIOTalonFX.java

**File:** `subsystems/flywheel/FlywheelIOTalonFX.java`
**Severity:** 🔴 CRITICAL + 🟠 HIGH + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Followers 2 & 3 set to `Opposed` — two of four motors fight the lead

**Lines:** 54–56

### Problem
```java
m_flywheelFollowMotor1.setControl(new Follower(lead, MotorAlignmentValue.Aligned));   // ✓
m_flywheelFollowMotor2.setControl(new Follower(lead, MotorAlignmentValue.Opposed));  // ✗
m_flywheelFollowMotor3.setControl(new Follower(lead, MotorAlignmentValue.Opposed));  // ✗
```

A flywheel driven by four Krakens on the same shaft **cannot** have two motors spinning in the opposite direction. `MotorAlignmentValue.Opposed` makes the follower rotate opposite to the lead — motors 2 and 3 will actively **fight the lead and motor 1**, causing:
- Massive current draw (likely tripping breakers)
- Net torque far below the intended 4-motor output
- Potential motor damage from opposing loads

### Fix

**Option A** — If all motors are on the same shaft driving in the same direction:
```java
m_flywheelFollowMotor1.setControl(new Follower(lead, MotorAlignmentValue.Aligned));
m_flywheelFollowMotor2.setControl(new Follower(lead, MotorAlignmentValue.Aligned));
m_flywheelFollowMotor3.setControl(new Follower(lead, MotorAlignmentValue.Aligned));
```

**Option B** — If motors 2 & 3 are physically wired reversed (some teams do this for packaging):
```java
// Keep Opposed but verify the physical wiring matches:
m_flywheelFollowMotor2.setControl(new Follower(lead, MotorAlignmentValue.Opposed));
m_flywheelFollowMotor3.setControl(new Follower(lead, MotorAlignmentValue.Opposed));
// ⚠️ Verify by spinning the flywheel by hand and checking all motors rotate the same way
```

**You must physically verify which option is correct.** Trust the physical wiring.

---

## 🔴 BUG 2 — Followers never configured (no current limits, no neutral mode, no feedback)

**Lines:** 67 (only lead gets `apply()`)

### Problem
```java
// Only the lead motor gets configured:
m_flywheelLeadMotor.getConfigurator().apply(flywheelConfig);
// Followers get setControl(Follower(...)) but NEVER get apply(flywheelConfig)
```

Phoenix followers inherit the lead's **control output** (duty cycle / voltage), but they do NOT inherit:
- **Current limits** — each follower runs with default 80A stator limit (dangerous!)
- **Neutral mode** — followers default to whatever Phoenix ships (could be Coast)
- **Motor output configs** — inversion is inherited via `Follower`, but neutral mode is not

### Fix
Apply the same config to all four motors:
```java
CtreUtil.reportIfNotOk("Flywheel lead config", m_flywheelLeadMotor.getConfigurator().apply(flywheelConfig));
CtreUtil.reportIfNotOk("Flywheel follower 1 config", m_flywheelFollowMotor1.getConfigurator().apply(flywheelConfig));
CtreUtil.reportIfNotOk("Flywheel follower 2 config", m_flywheelFollowMotor2.getConfigurator().apply(flywheelConfig));
CtreUtil.reportIfNotOk("Flywheel follower 3 config", m_flywheelFollowMotor3.getConfigurator().apply(flywheelConfig));

// Then set up followers AFTER configs are applied:
m_flywheelFollowMotor1.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
m_flywheelFollowMotor2.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), /* verify */));
m_flywheelFollowMotor3.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), /* verify */));
```

---

## 🟠 BUG 3 — Redundant `getVelocity().refresh()` after `BaseStatusSignal.refreshAll`

**Lines:** 72–78

### Problem
```java
BaseStatusSignal.refreshAll(m_FlywheelVelocity, ...);    // batch refresh all signals
inputs.velocityRPM = (m_flywheelLeadMotor.getVelocity().refresh().getValueAsDouble() * 60);
```

`m_FlywheelVelocity` is already cached and refreshed via `BaseStatusSignal.refreshAll`. The call `m_flywheelLeadMotor.getVelocity()` creates/fetches a **new** StatusSignal and `.refresh()` does a second network round-trip. This:
- Doubles the CAN bus traffic per cycle
- Creates a second fetch that may return slightly different values

### Fix
```java
BaseStatusSignal.refreshAll(m_FlywheelVelocity, /* ... other signals ... */);
inputs.velocityRPM = m_FlywheelVelocity.getValueAsDouble() * 60;
// Use the cached signal directly, don't re-fetch
```

---

## 🟡 BUG 4 — `resolveTargetRPM` uses hardcoded 10000 RPM with TODO comment

**File:** `subsystems/flywheel/Flywheel.java:91–95`

### Problem
```java
//shoot is NOT 10000 rpm  ← TODO marker
case ACTIVE -> 10000;
```

The comment indicates the 10000 RPM is a placeholder. The actual target should vary by distance to the target (using a lookup table or calculated formula). Running the flywheel at a fixed 10000 RPM regardless of distance will result in inconsistent shot accuracy.

### Fix
Replace with a distance-based lookup table:
```java
case ACTIVE -> getFlywheelTargetRPM(distanceToTarget);

// Lookup table:
private double getFlywheelTargetRPM(double distanceMeters) {
    // Interpolate from characterized data
    // e.g. LinearInterpolator or InterpolatingDoubleTreeBuffer
    return kFlywheelRPMLookupTable.get(distanceMeters);
}
```
