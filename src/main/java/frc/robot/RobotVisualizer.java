package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeConstants;

/**
 * Publishes both the 2D mechanism drawing and the articulated 3D CAD poses.
 *
 * <p>Every public entry point is a no-op on a real robot ({@link RobotBase#isSimulation()} is
 * false) — callers can call these unconditionally from subsystem code and pay nothing on the RIO.
 */
public final class RobotVisualizer {
    private RobotVisualizer() {
    }

    private static final boolean IS_SIM = RobotBase.isSimulation();

    // --- AdvantageScope 3D CAD visualization ---
    // These four poses map, in order, to model_0.glb through model_3.glb.
    private static final int INTAKE_COMPONENT = 0;
    private static final int TURRET_COMPONENT = 2;
    private static final int HOOD_COMPONENT = 3;
    private static final int HOOK_COMPONENT = 1;

    private static final Pose3d INTAKE_LINEAR_LOCATION = new Pose3d(
            Units.inchesToMeters(-10.239),
            0.0,
            Units.inchesToMeters(0.0),
            new Rotation3d(Units.degreesToRadians(180.0), 0.0, 0.0));
    private static final Pose3d HOOD_LOCATION = new Pose3d(
            Units.inchesToMeters(3.0),
            0.0,
            Units.inchesToMeters(20.0),
            new Rotation3d(Units.degreesToRadians(180.0), 0.0, 0.0));
    private static final Pose3d TURRET_LOCATION = new Pose3d(
            Units.inchesToMeters(0.0),
            0.0,
            Units.inchesToMeters(22.0),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)));
    private static final Pose3d HOOK_LOCATION = new Pose3d(
            Units.inchesToMeters(0.0),
            0.0,
            Units.inchesToMeters(4.375),
            new Rotation3d(Units.degreesToRadians(0.0), 0.0, 0.0));
    // Do all of the locations XZ before hand
    private static final Pose3d[] COMPONENTS = {
            INTAKE_LINEAR_LOCATION,
            HOOK_LOCATION,
            TURRET_LOCATION,
            HOOD_LOCATION
    };
    private static final StructArrayPublisher<Pose3d> COMPONENTS_PUBLISHER = IS_SIM
            ? NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Visualizer/Components", Pose3d.struct)
                    .publish()
            : null;

    private static double intakeAngleRadians;
    private static double intakeExtensionMeters;
    private static double turretAngleRadians;
    private static double hoodAngleRadians;
    private static double hookAngleRadians;

    /**
     * Returns a snapshot so callers cannot accidentally modify the published array.
     */
    public static Pose3d[] getComponents() {
        return COMPONENTS.clone();
    }

    /** Updates telescoping intake travel. The input is meters. No-op on a real robot. */
    public static void updateIntakeExtension(double extensionMeters) {
        if (!IS_SIM) {
            return;
        }
        intakeExtensionMeters = extensionMeters;
        updateIntakePose();
    }

    private static void updateIntakePose() {
        COMPONENTS[INTAKE_COMPONENT] = INTAKE_LINEAR_LOCATION.transformBy(
                new Transform3d(
                        new Translation3d(intakeExtensionMeters, 0.0,
                                (intakeExtensionMeters
                                        * Math.sin(Math.toRadians(IntakeConstants.kIntakeAngleDegrees)))),
                        new Rotation3d(0.0, 0.0, 0.0)));

        publishComponents();
    }

    /** Updates the turret yaw. The input is radians. No-op on a real robot. */
    public static void updateTurret(double angleRadians) {
        if (!IS_SIM) {
            return;
        }
        turretAngleRadians = Math.IEEEremainder(angleRadians, 2.0 * Math.PI);
        updateTurretAndHoodPoses();
    }

    /** Updates the hood pitch. The input is radians. No-op on a real robot. */
    public static void updateHood(double angleRadians) {
        if (!IS_SIM) {
            return;
        }
        hoodAngleRadians = -angleRadians;
        updateTurretAndHoodPoses();
    }

    private static void updateTurretAndHoodPoses() {
        Rotation3d turretRotation = new Rotation3d(0.0, 0.0, turretAngleRadians);
        COMPONENTS[TURRET_COMPONENT] = TURRET_LOCATION.transformBy(
                new Transform3d(Translation3d.kZero, turretRotation));
        COMPONENTS[HOOD_COMPONENT] = HOOD_LOCATION
                .rotateAround(TURRET_LOCATION.getTranslation(), turretRotation)
                .transformBy(new Transform3d(
                        Translation3d.kZero,
                        new Rotation3d(0.0, hoodAngleRadians, 0.0)));
        publishComponents();
    }

    /** Updates the hook pitch. The input is radians. No-op on a real robot. */
    public static void updateHook(double angleRadians) {
        if (!IS_SIM) {
            return;
        }
        hookAngleRadians += angleRadians % (2.0 * Math.PI);
        COMPONENTS[HOOK_COMPONENT] = HOOK_LOCATION.transformBy(
                new Transform3d(
                        Translation3d.kZero,
                        new Rotation3d(0.0, 0.0, -hookAngleRadians)));
        publishComponents();
    }

    private static void publishComponents() {
        COMPONENTS_PUBLISHER.set(COMPONENTS);
    }

    // --- WPILib Mechanism2d visualization ---
    private static final double BASE_X = Units.feetToMeters(3);
    private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);

    public static final Mechanism2d MECH_VISUALIZER = new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));

    private static final MechanismRoot2d DRIVETRAIN_ROOT = MECH_VISUALIZER.getRoot(
            "drivetrain-root", BASE_X, Units.inchesToMeters(7.5));
    private static final MechanismRoot2d HOOD_BASE = MECH_VISUALIZER.getRoot(
            "hood-base", BASE_X, Units.inchesToMeters(18.5));
    private static final MechanismRoot2d INTAKE_LINEAR_BASE = MECH_VISUALIZER.getRoot(
            "intake-linear-base",
            BASE_X + Units.inchesToMeters(11.5),
            Units.inchesToMeters(9.5));
    private static final MechanismRoot2d TURRET_BASE = MECH_VISUALIZER.getRoot(
            "turret-base",
            BASE_X - Units.inchesToMeters(8),
            Units.inchesToMeters(10));

    private static final MechanismLigament2d BACK_DRIVETRAIN_HALF = new MechanismLigament2d(
            "drive-back", Units.inchesToMeters(14), 180, 4, ORANGE);
    private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF = new MechanismLigament2d(
            "drive-front", Units.inchesToMeters(14), 0, 4, ORANGE);

    private static boolean isSetup;

    /** Call once during robot initialization. No-op on a real robot. */
    public static void setupVisualizer() {
        if (!IS_SIM || isSetup) {
            return;
        }
        isSetup = true;
        DRIVETRAIN_ROOT.append(BACK_DRIVETRAIN_HALF);
        DRIVETRAIN_ROOT.append(FRONT_DRIVETRAIN_HALF);
        SmartDashboard.putData("Visualizer/Mechanism", MECH_VISUALIZER);
        publishComponents();
    }

    /** No-op on a real robot. */
    public static void addHood(MechanismLigament2d hood) {
        if (!IS_SIM) {
            return;
        }
        HOOD_BASE.append(hood);
    }

    /** No-op on a real robot. */
    public static void addIntake(MechanismLigament2d intake) {
        if (!IS_SIM) {
            return;
        }
        INTAKE_LINEAR_BASE.append(intake);
    }

    /** No-op on a real robot. */
    public static void addTurret(MechanismLigament2d turret) {
        if (!IS_SIM) {
            return;
        }
        TURRET_BASE.append(turret);
    }
}
