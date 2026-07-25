# OffSeasonBot-2026 Code Review — Bug Report Index

**Reviewed:** 2026-07-25
**Reviewer:** Hermes Agent (automated)
**Project:** `C:\Users\evan\OffSeasonBot-2026` (WPILib 2026, Java 17, Phoenix 6)

---

## Summary

A comprehensive code review found **48+ issues** across the codebase. Bug report markdown files have been placed next to their corresponding source files. Below is an index and priority-ordered fix list.

---

## Bug Report Files

| File | Location | Severity |
|------|----------|----------|
| [TurretIOTalonFX_BUGS.md](../subsystems/turret/TurretIOTalonFX_BUGS.md) | `subsystems/turret/` | 🔴 CRITICAL |
| [Turret_BUGS.md](../subsystems/turret/Turret_BUGS.md) | `subsystems/turret/` | 🔴 CRITICAL |
| [IntakeIOTalonFX_BUGS.md](../subsystems/intake/IntakeIOTalonFX_BUGS.md) | `subsystems/intake/` | 🔴 CRITICAL |
| [IntakeIOSimTalonFX_BUGS.md](../subsystems/intake/IntakeIOSimTalonFX_BUGS.md) | `subsystems/intake/` | 🟡 MEDIUM |
| [DyeRotor_BUGS.md](../subsystems/dyerotor/DyeRotor_BUGS.md) | `subsystems/dyerotor/` | 🔴 CRITICAL |
| [FlywheelIOTalonFX_BUGS.md](../subsystems/flywheel/FlywheelIOTalonFX_BUGS.md) | `subsystems/flywheel/` | 🔴 CRITICAL |
| [Hood_BUGS.md](../subsystems/hood/Hood_BUGS.md) | `subsystems/hood/` | 🟠 HIGH |
| [HoodIOSimTalonFX_BUGS.md](../subsystems/hood/HoodIOSimTalonFX_BUGS.md) | `subsystems/hood/` | 🔴 CRITICAL (sim) |
| [Vision_BUGS.md](../subsystems/vision/apriltag/Vision_BUGS.md) | `subsystems/vision/apriltag/` | 🔴 CRITICAL |
| [Superstructure_BUGS.md](../subsystems/Superstructure_BUGS.md) | `subsystems/` | 🟠 HIGH |
| [AutoAlign_BUGS.md](../util/AutoAlign_BUGS.md) | `util/` | 🔴 CRITICAL |
| [RobotContainer_BUGS.md](../RobotContainer_BUGS.md) | project root | 🟠 HIGH |
| [Util_BUGS.md](../util/Util_BUGS.md) | `util/` | 🟠 HIGH |
| [TunerConstants_BUGS.md](../generated/TunerConstants_BUGS.md) | `generated/` | 🟡 MEDIUM |

---

## 🔴 CRITICAL BUGS (will break the robot on the field)

| # | Bug | File:Line | Fix |
|---|-----|-----------|-----|
| 1 | Turret CAN bus not specified — motor unreachable | `TurretIOTalonFX.java:28` | Add `Constants.CANBuses.LowerBus` (or `UpperBus`) to constructor |
| 2 | Intake extension config applied to follower, not lead | `IntakeIOTalonFX.java:125` | Change `m_extensionFollowerMotor` → `m_extensionLeadMotor` in `apply()`, move Slot0 before apply |
| 3 | DyeRotor indexer RPM passed to voltage request | `DyeRotor.java:109` | Change `setIndexVoltage` → `setIndexVelocityRPM` with `VelocityVoltage` |
| 4 | Turret `requestedAngle` never written — always 0° | `Turret.java:10,69` | Add `setRequestedAngle(double)` setter, call from Superstructure |
| 5 | Flywheel followers 2&3 set to `Opposed` — fighting lead | `FlywheelIOTalonFX.java:55-56` | Verify physical mounting, change to `Aligned` if same-side |
| 6 | Hood/Turret sim position off by factor of kReduction | `HoodIOSimTalonFX.java:49`, `TurretIOSimTalonFX.java:44` | Multiply by `kReduction` before `setRawRotorPosition` |
| 7 | AprilTag vision fully unwired — never instantiated | `RealATVision.java`, `RobotContainer.java` | Instantiate vision, schedule `periodic()`, call `addVisionMeasurement` |
| 8 | AprilTag NaN enters pose estimator when tagCount=0 | `AprilTagModule.java:180` | Guard `avgAmbiguity /= tagCount` with `if (tagCount > 0)` |
| 9 | AutoAlign passes robot-relative to field-centric request | `AutoAlign.java:211-216` | Convert Autopilot output to field-relative, or use `RobotCentric` |
| 10 | AutoAlign hardcoded to Blue alliance | `AutoAlign.java:73` | Branch on `DriverStation.getAlliance()` |

---

## 🟠 HIGH SEVERITY (silent failures / significantly wrong behavior)

| # | Bug | File:Line |
|---|-----|-----------|
| 11 | Hood/Turret all real PID gains = 0 — will sag | `Hood.java:21-31`, `Turret.java:18-23` |
| 12 | Flywheel followers never configured (no current limits) | `FlywheelIOTalonFX.java` |
| 13 | Superstructure commands missing `addRequirements` | `Superstructure.java:62-104` |
| 14 | Superstructure subsystem fields are `public` mutable | `Superstructure.java:36-40` |
| 15 | Autos + RobotVisualizer are dead code | `RobotContainer.java`, `RobotVisualizer.java` |
| 16 | No CTreUtil status code checking on any `apply()` | All IO files |
| 17 | AutoAlign heading PID `5,0,0` — no D term, oscillation | `AutoAlign.java:75` |
| 18 | AutoAlign `DEFAULT_MAX_VELOCITY=5.5` > physical max 4.39 | `AutoAlign.java:28` |
| 19 | Two competing heading controllers in AutoAlign | `AutoAlign.java:72-75, 211-216` |
| 20 | `AutoAlignFixedHeading.m_realTarget` NPE before `execute()` | `AutoAlignFixedHeading.java:109` |
| 21 | `SwerveDriveStateLogger` no null checks → NPE | `SwerveDriveStateLogger.java:20-26` |
| 22 | LimelightHelpers timestamp not FPGA time | `LimelightHelpers.java:911-931` |

---

## 🟡 MEDIUM / LOW SEVERITY

See individual bug report files for the full list of medium and low severity issues including: code smells, dead code, naming inconsistencies, missing null guards, and physical verification warnings.

---

## Top-Priority Fix Order

1. **Turret CAN bus** (#1) — without this the turret doesn't exist
2. **Intake extension config** (#2) — extension won't work
3. **DyeRotor indexer units** (#3) — indexer will overshoot
4. **Turret angle setter** (#4) — turret always points at 0°
5. **Flywheel follower alignment** (#5) — motors fighting
6. **Sim position fix** (#6) — sim is unusable
7. **NaN guard in AprilTag** (#8) — corrupts pose estimator
8. **Wire vision** (#7) — no AprilTag pose correction
9. **AutoAlign frame fix** (#9, #10) — drives wrong direction
10. **Tune PID gains** (#11, #17, #18) — mechanisms won't hold position
11. **Clean dead code** (#15) — Autos, RobotVisualizer, TriggerCommand
12. **CtreUtil status checking** (#16) — config failures silent

---

## Key Architectural Notes

- **Vendor deps**: Phoenix6-26.3, ChoreoLib2026, BLine-Lib v0.9.1 (uses package `frc.robot.lib.BLine` — unusual but verified from jar), Autopilot 1.6.1 (therekrab), WPILibNewCommands
- **No PhotonVision vendor jar** — LimelightHelpers is hand-vendored (v1.14, requires LLOS 2026.0+)
- **Build**: Java 17 (WPILib 2026 supports Java 21 — consider upgrading)
- **CAN bus layout**: Two CANivores — `UpperBus` (Intake, potentially Flywheel) and `LowerBus` (DyeRotor, potentially Turret/Flywheel). Verify physical wiring.
