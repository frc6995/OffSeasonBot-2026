package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
  private static final double BASE_X = Units.feetToMeters(3);
  private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);
  private static final Color8Bit BLUE = new Color8Bit(52, 137, 235);

  public static final Mechanism2d MECH_VISUALIZER = new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));

  private static final MechanismRoot2d DRIVETRAIN_ROOT = MECH_VISUALIZER.getRoot("drivetrain-root", BASE_X,
      Units.inchesToMeters(7.5));

  private static final MechanismRoot2d HOOD_BASE = MECH_VISUALIZER.getRoot("hood-base", BASE_X,
      Units.inchesToMeters(18.5));

  private static final MechanismRoot2d INTAKE_PIVOT_BASE = MECH_VISUALIZER.getRoot(
      "intake-pivot-base",
      BASE_X + Units.inchesToMeters(11.5),
      Units.inchesToMeters(9.5));

  private static final MechanismRoot2d TURRET_BASE = MECH_VISUALIZER.getRoot(
      "turret-base",
      BASE_X - Units.inchesToMeters(8),
      Units.inchesToMeters(10));

  private static final MechanismLigament2d BACK_DRIVETRAIN_HALF = new MechanismLigament2d("drive-back",
      Units.inchesToMeters(14), 180, 4, ORANGE);
  private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF = new MechanismLigament2d("drive-front",
      Units.inchesToMeters(14), 0, 4, ORANGE);

  public static void setupVisualizer() {
    DRIVETRAIN_ROOT.append(BACK_DRIVETRAIN_HALF);
    DRIVETRAIN_ROOT.append(FRONT_DRIVETRAIN_HALF);
    SmartDashboard.putData("Visualizer/Mechanism", MECH_VISUALIZER);
  }

  // --- Subsystem attachment points ---

  public static void addHood(MechanismLigament2d hood) {
    HOOD_BASE.append(hood);
  }

  public static void addIntake(MechanismLigament2d intake) {
    INTAKE_PIVOT_BASE.append(intake);
  }

  // public static void addDyeRotor(MechanismLigament2d rotor) {
  //   DYE_ROTOR_BASE.append(rotor);
  // }
  // public static void addFlywheel(MechanismLigament2d flywheel) {
  //   FLYWHEEL_BASE.append(flywheel);
  // }

  public static void addTurret(MechanismLigament2d turret) {
    TURRET_BASE.append(turret);
  }

}