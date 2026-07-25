# Bug Report: TurretIOTalonFX.java

**File:** `subsystems/turret/TurretIOTalonFX.java`
**Severity:** 🔴 CRITICAL + 🟠 HIGH
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Turret motor CAN bus not specified (motor unreachable)

**Lines:** 27–28

### Problem
```java
//need to specify upper or lower CAN bus
protected final TalonFX m_turretMotor = new TalonFX(kCANID);
```

The `TalonFX(int deviceID)` constructor without a CAN bus name uses the **default CAN bus** (empty string `""`), which corresponds to the first CANivore detected by the system — typically the RIO's internal CAN bus. Every other motor in this project explicitly specifies a CAN bus:

- Intake motors → `Constants.CANBuses.UpperBus`
- DyeRotor motors → `Constants.CANBuses.LowerBus`
- Flywheel motors → likely `LowerBus` or `UpperBus`

If the turret motor is physically wired to `LowerBus` or `UpperBus` (which is almost certainly the case since the project uses two CANivores), the motor will **never be reachable** from code. `m_turretMotor.isConnected()` will return `false`, `getPosition()` will return stale/default values, and `setControl(...)` will have no effect. **The turret will not move at all.**

### Fix
Determine which physical CAN bus the turret motor is wired to, then:

```java
protected final TalonFX m_turretMotor = new TalonFX(kCANID, Constants.CANBuses.LowerBus);
// or Constants.CANBuses.UpperBus — verify against the physical wiring
```

---

## 🟠 BUG 2 — CTRE `apply()` return value never checked

**Lines:** 90

### Problem
```java
m_turretMotor.getConfigurator().apply(config);
```

The `apply()` method returns a `StatusCode` indicating success or failure. During brownouts, CAN bus congestion, or misconfiguration, `apply()` can silently fail and the motor runs with Phoenix defaults (no PID, no Motion Magic, no current limits, no soft limits). The code even has a TODO comment:

```java
//TODO replace this with CtreUtil reportIfNotOk
```

### Fix
Use the `CtreUtil` helper that already exists in the project:

```java
CtreUtil.reportIfNotOk("Turret config", m_turretMotor.getConfigurator().apply(config));
```

---

## 🟡 BUG 3 — Motion Magic acceleration and cruise velocity set to 0

**Lines:** 84–87

### Problem
```java
config.MotionMagic = 
    new MotionMagicConfigs()
        .withMotionMagicAcceleration(0)
        .withMotionMagicCruiseVelocity(0);
```

With acceleration and cruise velocity both set to 0, Motion Magic **cannot move the motor**. The motor will try to reach the target position instantly (infinite acceleration) but clamped to 0 cruise velocity — effectively the motor will not respond to `MotionMagicVoltage` position requests at all, or will behave unpredictably.

### Fix
Set these to appropriate values for the turret mechanism:

```java
config.MotionMagic = 
    new MotionMagicConfigs()
        .withMotionMagicAcceleration(kMotionMagicAcceleration) // e.g. 200 rot/s²
        .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity); // e.g. 20 rot/s
```

Tune these values by testing the turret's physical limits.

---

## 🟡 BUG 4 — `disable()` uses `set(0)` mixing control modes

**Lines:** 133–135

### Problem
```java
public void disable() {
    m_turretMotor.set(0);
}
```

`set(0)` applies duty-cycle (percent output) at 0%. While this does stop the motor, it mixes control modes — the rest of the code uses `MotionMagicVoltage`. Prefer `setControl(new Neutral())` or `stopMotor()` to cleanly release the controller.

### Fix
```java
public void disable() {
    m_turretMotor.stopMotor();
}
```

---

## 🟡 BUG 5 — Soft limit warning text hardcoded to `[-360, 360]`

**Lines:** 113–118

### Problem
```java
if (clampedAngle != angle) {
    DriverStation.reportWarning(
      "Angle requested outside of range [-360, 360], clamped to %f degrees"
        .formatted(clampedAngle),
        false  
    );
}
```

The warning text says `[-360, 360]` but the actual limits come from `kMinAngle` and `kMaxAngle`. If those constants are changed, the warning text will be wrong and misleading.

### Fix
```java
DriverStation.reportWarning(
    "Angle requested outside of range [%f, %f], clamped to %f degrees"
        .formatted(kMinAngle, kMaxAngle, clampedAngle),
    false
);
```
