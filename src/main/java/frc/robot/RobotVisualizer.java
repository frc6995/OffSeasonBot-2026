package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
  private static final double BASE_X = Units.feetToMeters(3);
  private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);
  private static final Color8Bit BLUE = new Color8Bit(52, 137, 235);

  public static final Mechanism2d MECH_VISUALIZER = new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));

  // --- Roots (fixed mount points, robot-relative, side view: X=fwd/back, Y=up)
  // ---
  private static final MechanismRoot2d DRIVETRAIN_ROOT = MECH_VISUALIZER.getRoot("drivetrain-root", BASE_X,
      Units.inchesToMeters(7.5));

  // TODO: set real mount coordinates (inches from robot center, ground-up) once
  // measured
  private static final MechanismRoot2d SHOOTER_BASE = MECH_VISUALIZER.getRoot("shooter-base", BASE_X,
      Units.inchesToMeters(18.5));

  private static final MechanismRoot2d INTAKE_PIVOT_BASE = MECH_VISUALIZER.getRoot(
      "intake-pivot-base",
      BASE_X + Units.inchesToMeters(11.5),
      Units.inchesToMeters(9.5));

  private static final MechanismRoot2d DYE_ROTOR_BASE = MECH_VISUALIZER.getRoot(
      "dye-rotor-base",
      BASE_X - Units.inchesToMeters(8), // TODO: real offset
      Units.inchesToMeters(10));

  // --- Fixed drivetrain footprint ---
  private static final MechanismLigament2d BACK_DRIVETRAIN_HALF = new MechanismLigament2d("drive-back",
      Units.inchesToMeters(14), 180, 4, ORANGE);
  private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF = new MechanismLigament2d("drive-front",
      Units.inchesToMeters(14), 0, 4, ORANGE);

  // Tracks the shooter's ligament so the hood can nest onto it (hood angle is
  // relative to the shooter body, which itself pivots relative to the robot).
  private static MechanismLigament2d shooterLigament;

  public static void setupVisualizer() {
    DRIVETRAIN_ROOT.append(BACK_DRIVETRAIN_HALF);
    DRIVETRAIN_ROOT.append(FRONT_DRIVETRAIN_HALF);
    SendableRegistry.add(MECH_VISUALIZER, "Visualizer/Mechanism");
  }

  // --- Subsystem attachment points ---

  public static void addShooterPivot(MechanismLigament2d shooter) {
    SHOOTER_BASE.append(shooter);
    shooterLigament = shooter;
  }

  public static void addHood(MechanismLigament2d hood) {
    if (shooterLigament == null) {
      throw new IllegalStateException(
          "addShooterPivot() must be called before addHood() - hood mounts on the shooter");
    }
    shooterLigament.append(hood);
  }

  public static void addIntake(MechanismLigament2d intake) {
    INTAKE_PIVOT_BASE.append(intake);
  }

  public static void addDyeRotor(MechanismLigament2d dyerotor) {
    DYE_ROTOR_BASE.append(dyerotor);
  }

}