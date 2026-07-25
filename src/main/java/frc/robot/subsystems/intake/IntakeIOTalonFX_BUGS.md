# Bug Report: IntakeIOTalonFX.java

**File:** `subsystems/intake/IntakeIOTalonFX.java`
**Severity:** 🔴 CRITICAL + 🟠 HIGH + 🟡 MEDIUM
**Reviewed:** 2026-07-25

---

## 🔴 BUG 1 — Extension config applied to FOLLOWER, not LEAD motor (lead is unconfigured)

**Lines:** 106–130

### Problem
```java
private void configureExtensionMotors() {
     TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput = ...;
    extensionConfig.CurrentLimits = ...;
    extensionConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kExtensionReduction);
    
    m_extensionFollowerMotor.setControl(new Follower(m_extensionLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));  // line 121

    extensionConfig.MotionMagic.withMotionMagicAcceleration(IntakeConstants.acceleration)
         .withMotionMagicCruiseVelocity(IntakeConstants.velocity);
    
    m_extensionFollowerMotor.getConfigurator().apply(extensionConfig);  // ← APPLIED TO FOLLOWER, NOT LEAD

    extensionConfig.Slot0                              // ← Slot0 mutated AFTER apply()
        .withKP(IntakeConstants.kExtensionP)
        .withKV(IntakeConstants.kExtensionV);          // ← NEVER APPLIED TO ANY MOTOR
}
```

This has **two compounding critical errors**:

1. **The `TalonFXConfiguration` is `apply()`-ed to `m_extensionFollowerMotor` instead of `m_extensionLeadMotor`.** The lead motor is never configured — it runs with Phoenix defaults: no PID gains, no Motion Magic, no current limits, no `SensorToMechanismRatio`. When `setExtensionPosition()` calls `m_extensionLeadMotor.setControl(m_extensionRequest.withPosition(...))`, the lead motor has no Slot0 gains and no feedback configuration, so Motion Magic will not function correctly.

2. **`Slot0.withKP(kExtensionP).withKV(kExtensionV)` at lines 127–129 happens AFTER `apply()`.** These gains are never written to any motor. They are silently dropped. The extension PID gains are completely absent from the hardware.

**Net effect:** The intake extension does not act as a controlled position mechanism. It will either sit dead or move with default P only, ignoring the tuned gains.

### Fix
```java
private void configureExtensionMotors() {
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    extensionConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.kExtensionStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.kExtensionSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);

    extensionConfig.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kExtensionReduction);

    extensionConfig.MotionMagic
        .withMotionMagicAcceleration(IntakeConstants.acceleration)
        .withMotionMagicCruiseVelocity(IntakeConstants.velocity);

    // Set Slot0 BEFORE apply()
    extensionConfig.Slot0
        .withKP(IntakeConstants.kExtensionP)
        .withKV(IntakeConstants.kExtensionV);

    // Apply to LEAD motor (not follower!)
    CtreUtil.reportIfNotOk("Extension lead config",
        m_extensionLeadMotor.getConfigurator().apply(extensionConfig));

    // Follower just needs to follow the lead
    m_extensionFollowerMotor.setControl(
        new Follower(m_extensionLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
}
```

---

## 🟠 BUG 2 — Roller follower configuration order: `setControl` before `apply`

**Lines:** 99–103

### Problem
```java
rollerConfig.Feedback =
    new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kRollerReduction);
    m_rollerFollowerMotor.setControl(new Follower(m_rollerLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
m_rollerLeadMotor.getConfigurator().apply(rollerConfig);
```

The follower is configured with `setControl(new Follower(...))` BEFORE the lead motor's config is applied. While the `Follower` control request only needs the lead's device ID (which doesn't change), best practice is to configure the lead first, then set up the follower. Also note the indentation on line 101 is wrong — the `setControl` call looks like it's inside the `FeedbackConfigs` chain but it's not.

### Fix
```java
// Configure lead first
m_rollerLeadMotor.getConfigurator().apply(rollerConfig);
// Then set up follower
m_rollerFollowerMotor.setControl(
    new Follower(m_rollerLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
```

---

## 🟠 BUG 3 — Follower motor alignment `Opposed` — verify physical mounting

**Lines:** 101, 121

### Problem
```java
m_rollerFollowerMotor.setControl(new Follower(m_rollerLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
m_extensionFollowerMotor.setControl(new Follower(m_extensionLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
```

`MotorAlignmentValue.Opposed` means the follower spins in the **opposite direction** from the lead. This is correct **only if** the follower motor is physically mirrored on the opposite side of the mechanism (so that opposite rotation produces the same physical motion). If both motors are on the same shaft or driving the same gear train from the same side, `Opposed` will make them fight each other — potentially blowing breakers.

### Fix
Verify the physical mounting of the roller and extension follower motors:
- If the follower is mirrored on the opposite side → `Opposed` is correct.
- If the follower is co-located on the same side → change to `MotorAlignmentValue.Aligned`.

---

## 🟡 BUG 4 — No CTRE `apply()` return value checking

**Lines:** 84, 102

### Problem
```java
m_kickerMotor.getConfigurator().apply(kickConfig);       // line 84
m_rollerLeadMotor.getConfigurator().apply(rollerConfig);  // line 102
```

None of the `apply()` calls check the `StatusCode` return value. If the config fails (brownout, CAN congestion), the motors silently run with defaults.

### Fix
Wrap all `apply()` calls:
```java
CtreUtil.reportIfNotOk("Kicker config", m_kickerMotor.getConfigurator().apply(kickConfig));
CtreUtil.reportIfNotOk("Roller config", m_rollerLeadMotor.getConfigurator().apply(rollerConfig));
```

---

## 🟡 BUG 5 — FlywheelIOTalonFX pattern repeated: followers never configured

The roller and extension follower motors never get their own `TalonFXConfiguration.apply()`. While Phoenix followers inherit the lead's control output, they do NOT inherit:
- Current limits (each motor needs its own current limit config)
- Neutral mode
- Motor output settings

### Fix
Apply a basic config to each follower motor as well:
```java
TalonFXConfiguration followerConfig = new TalonFXConfiguration();
followerConfig.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
followerConfig.CurrentLimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(IntakeConstants.kRollerStatorCurrentLimit)
    .withStatorCurrentLimitEnable(true);
m_rollerFollowerMotor.getConfigurator().apply(followerConfig);
```
