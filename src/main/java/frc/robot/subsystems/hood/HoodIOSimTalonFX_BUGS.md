# Bug Report: HoodIOSimTalonFX.java & TurretIOSimTalonFX.java

**Files:**
- `subsystems/hood/HoodIOSimTalonFX.java`
- `subsystems/turret/TurretIOSimTalonFX.java`

**Severity:** 🔴 CRITICAL (sim only — but sim is used for development and validation)
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Sim `setRawRotorPosition` receives MECHANISM rotations, not ROTOR rotations (off by factor of kReduction)

### HoodIOSimTalonFX.java
**Line:** 49

```java
simState.setRawRotorPosition(angleToRotations(hoodPosition));
```

### TurretIOSimTalonFX.java
**Line:** 44

```java
simState.setRawRotorPosition(angleToRotations(turretPosition));
```

### Problem

`angleToRotations(angle)` returns `angle / 360.0`, which gives **mechanism rotations** (the angle in degrees converted to rotations of the output shaft).

`simState.setRawRotorPosition(double)` expects **raw ROTOR rotations** — the motor's internal rotor position before the gear reduction.

However, Phoenix 6 has `FeedbackConfigs.withSensorToMechanismRatio(kReduction)` configured (e.g., `kReduction = 70.29` for Hood). Phoenix's sensor chain works as:

```
Mechanism Position = Raw Rotor Position / kReduction
```

So when you call `setRawRotorPosition(angleToRotations(42.5°))`:
1. You pass `0.118` mechanism rotations as the raw rotor position
2. Phoenix divides by `kReduction` (70.29): reports `0.118 / 70.29 = 0.00168` mechanism rotations
3. `rotationsToAngle()` converts to degrees: `0.00168 * 360 = 0.6°`
4. The controller sees the hood at **0.6° instead of 42.5°**

The simulation model is broken by a factor of `kReduction`. The simulated hood/turret will never reach its target — the controller will keep commanding because the reported position is 70× too small.

### Fix

**Option A** — Multiply by kReduction before setting (straightforward):
```java
// HoodIOSimTalonFX.java line 49:
simState.setRawRotorPosition(angleToRotations(hoodPosition) * Hood.HoodConstants.kReduction);

// TurretIOSimTalonFX.java line 44:
simState.setRawRotorPosition(angleToRotations(turretPosition) * Turret.TurretConstants.kReduction);
```

**Option B** — Use Phoenix's `setMechanismPosition` if available (cleaner):
```java
// If available in your Phoenix 6 version:
simState.setMechanismPosition(angleToRotations(hoodPosition));
```

---

## 🟠 BUG 2 — Hood sim update order: physics step BEFORE input voltage set

**File:** `subsystems/hood/HoodIOSimTalonFX.java`
**Lines:** 38, 45

### Problem
```java
m_HoodSim.update(0.02);                          // line 38 — physics step FIRST
...
m_HoodSim.setInputVoltage(appliedVolts);         // line 45 — voltage set AFTER
```

The physics model updates **before** the input voltage for this tick is applied. This means the physics always uses the **previous tick's voltage**, introducing a one-cycle lag. Compare with `TurretIOSimTalonFX.java` which does it in the correct order (set voltage → read state → update physics).

### Fix
Reorder to match Turret's pattern:
```java
// 1. Set supply voltage
simState.setSupplyVoltage(RobotController.getBatteryVoltage());

// 2. Get applied voltage
double appliedVolts = simState.getMotorVoltageMeasure().baseUnitMagnitude();

// 3. Set input to simulation
m_HoodSim.setInputVoltage(appliedVolts);

// 4. Set rotor position (FIX: multiply by kReduction)
simState.setRawRotorPosition(angleToRotations(hoodPosition) * Hood.HoodConstants.kReduction);

// 5. Read inputs
inputs.angle = hoodPosition;
inputs.appliedVolts = appliedVolts;
// ...

// 6. Update physics LAST
m_HoodSim.update(0.02);
```

---

## 🟡 BUG 3 — Sim motor type hardcoded as `KrakenX44` — verify against real hardware

**Files:**
- `HoodIOSimTalonFX.java:14` — `DCMotor.getKrakenX44(1)`
- `TurretIOSimTalonFX.java:11` — `DCMotor.getKrakenX44(1)`

### Problem

The sim uses `DCMotor.getKrakenX44(1)` (Kraken X44 motor model). If the actual hardware uses Kraken X60 motors (which are more powerful and have different torque constants), the simulation model will have incorrect torque characteristics — making sim performance not match real performance.

### Fix
Verify which motor type is physically installed:
```java
// If Kraken X60:
DCMotor.getKrakenX60(1);

// If Kraken X44 (or old Falcon 500):
DCMotor.getKrakenX44(1);  // current code — correct if X44
```

Also note: `setMotorType(TalonFXSimState.MotorType.KrakenX44)` is set in the turret sim's `configureSim()` but NOT in the hood sim. The hood sim should also call:
```java
simState.setMotorType(TalonFXSimState.MotorType.KrakenX44); // or KrakenX60
```
