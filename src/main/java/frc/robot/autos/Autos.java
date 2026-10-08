package frc.robot.autos;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CANRange;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.AllianceFlipUtil;

public class Autos {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Superstructure m_superstructure;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;
    private final CANRange m_canRange = new CANRange();

    // ============= BLINE PATHS =============
    /**
     * BLine trajectory paths for autonomous routines. Each Path references a
     * trajectory file (in lowercase)
     */

    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());
    public final Supplier<Pose2d> TRENCH_START_LEFT = AllianceFlipUtil.flipped(Depot1Path.getStartPose());
    private final Path Depot2Path = new Path("depot-2".toLowerCase());
    private final Path Depot3Path = new Path("depot-3".toLowerCase());

    private final Path LeftBump1Path = new Path("path-1".toLowerCase());
    private final Path LeftBump2Path = new Path("path-2".toLowerCase());
    private final Path RightBump1Path = mirroredCopy(LeftBump1Path);
    private final Path RightBump2Path = mirroredCopy(LeftBump2Path);
    private final Supplier<Pose2d> TRENCH_START_RIGHT =
        AllianceFlipUtil.flipped(RightBump1Path.getStartPose());

    private final Path Testcanrange = new Path("Test-canrange".toLowerCase());
    private final Path Testcanrange2 = new Path("Test-canrange2".toLowerCase());

    private final Supplier<Pose2d> TEST_START_CANRANGE = AllianceFlipUtil.flipped(Testcanrange.getStartPose());

    // Constructor
    public Autos(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.m_drivetrain = drivetrain;
        this.m_superstructure = superstructure;

        // ===== DEPOT AUTO EVENT TRIGGERS =====
        FollowPath.registerEventTrigger("intakeIdle", m_superstructure.requestIntakeIdle());
//        FollowPath.registerEventTrigger("startShooter", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("startShooting", Commands.parallel(
                m_superstructure.requestRobotScoring()));
        FollowPath.registerEventTrigger("startIntakingAgain", m_superstructure.requestIntakeMiniAgitate());
        FollowPath.registerEventTrigger("stopScoring",
                Commands.parallel(m_superstructure.requestRobotIdle(), superstructure.requestIntakeActive()));

        // ===== LEFT BUMP PATH 1 EVENT TRIGGERS =====
        FollowPath.registerEventTrigger("leftBumpStopIntake", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStartShooter", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("leftBumpStartShooting", m_superstructure.requestRobotScoring());
        FollowPath.registerEventTrigger("leftBumpStartAgitating", m_superstructure.requestIntakeMiniAgitate());
        FollowPath.registerEventTrigger("leftBumpStopAgitating", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStopShooting", m_superstructure.requestRobotIdle());

        // ===== LEFT BUMP PATH 2 EVENT TRIGGERS =====
        FollowPath.registerEventTrigger("leftBumpStopIntakeAgain", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStartShooterAgain", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("leftBumpStartShootingAgain", m_superstructure.requestRobotScoring());
        FollowPath.registerEventTrigger("leftBumpStartAgitatingAgain", m_superstructure.requestIntakeMiniAgitate());

        // ===== BLINE PATH FOLLOWING CONFIGURATION =====
        pathBuilder = new FollowPath.Builder(
                m_drivetrain, // Subsystem requirement
                m_drivetrain::getPose, // Supplier<Pose2d> - current robot pose from odometry
                m_drivetrain::getChassisSpeeds, // Supplier<ChassisSpeeds> (robot-relative), current velocities
                m_drivetrain::drive, // Consumer<ChassisSpeeds> (robot-relative), drivetrain command handler
                new PIDController(5.0, 0.0, 0.0), // Translation PID, minimizes remaining distance error
                new PIDController(7.0, 0.0, 0.0), // Rotation PID, minimizes heading error
                new PIDController(0.0, 0.0, 0.0) // Cross-track PID, minimizes perpendicular deviation
        )
                .withDefaultShouldFlip(); // Auto-flip trajectory when on red alliance

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============
    /** Registers all autonomous routines with the auto chooser. */
    private void registerAutos() {

        autos.put("Depot Auto",
                () -> auto(TRENCH_START_LEFT.get(), c -> {
                    Command Depot1 = pathBuilder.build(Depot1Path);
                    Command Depot2 = pathBuilder.build(Depot2Path);
                    Command Depot3 = pathBuilder.build(Depot3Path);

                    c.addCommands(untilCloseToWallAfterEvent(Depot1, "hubSensorActivation", 5)
                            .alongWith(m_superstructure.requestIntakeActive()));

                    c.addCommands(untilCloseToWallAfterEvent(Depot2, "depotSensorActivation", 7));

                    c.addCommands(Depot3);
                }));

        autos.put("Left Double Swipe Bump",
                () -> auto(TRENCH_START_LEFT.get(), c -> {
                    Command LeftBump1Cmd = pathBuilder.build(LeftBump1Path);
                    Command LeftBump2Cmd = pathBuilder.build(LeftBump2Path);

                    c.addCommands(LeftBump1Cmd.alongWith(m_superstructure.requestIntakeActive(),
                            m_superstructure.requestRobotIdle()));

                    c.addCommands(LeftBump2Cmd.alongWith(m_superstructure.requestIntakeActive()));
                }));

                 autos.put("Right Double Swipe Bump",
                () -> auto(TRENCH_START_RIGHT.get(), c -> {
                    Command RightBump1Cmd = pathBuilder.build(RightBump1Path);
                    Command RightBump2Cmd = pathBuilder.build(RightBump2Path);

                    c.addCommands(RightBump1Cmd.alongWith(m_superstructure.requestIntakeActive(),
                            m_superstructure.requestRobotIdle()));

                    c.addCommands(RightBump2Cmd.alongWith(m_superstructure.requestIntakeActive()));
                }));

        autos.put("Bline_Workshop_Test_Canrange",
                () -> auto(TEST_START_CANRANGE.get(), c -> {
                    Command canRangeTestAuto1 = pathBuilder.build(Testcanrange);
                    Command canRangeTestAuto2 = pathBuilder.build(Testcanrange2);

                    c.addCommands(untilCloseToWallAfterEvent(canRangeTestAuto1, "testActivation", 6));
                    c.addCommands((canRangeTestAuto2));
                }));

        // Register all autos with the chooser for driver station selection
        autos.forEach(autoChooser::addCmd);
    }

    /**
     * Runs a path command until it times out or the robot is close to a
     * wall but only after a specified event marker has fired in the
     * trajectory.
     *
     * @param path           the BLine path command to follow
     * @param eventKey       the path event marker name that must fire before sensor
     *                       termination is active
     * @param timeoutSeconds maximum time to allow the path to run (safety limit)
     * @return a command that follows the path with sensor-based early termination
     */
    private Command untilCloseToWallAfterEvent(Command path, String eventKey, double timeoutSeconds) {
        AtomicBoolean eventFired = new AtomicBoolean(false);
        FollowPath.registerEventTrigger(eventKey, () -> eventFired.set(true));

        return Commands.sequence(
                // Reset flag at start of command
                Commands.runOnce(() -> eventFired.set(false)),
                // Run path with timeout and terminate when event + sensor both trigger
                path.withTimeout(timeoutSeconds)
                        .until(() -> eventFired.get() && m_canRange.isCloseToWall()));
    }

    /**
     * Runs a throwaway BLine {@link FollowPath} through initialize()/execute()/isFinished()/end()
     * so its first-use class-loading cost is paid during construction rather than in the first
     * autonomous loops. Confirmed on-robot 2026-10-03 with per-command timing: on the first auto enable
     * after a code restart, path1.initialize() took 110 ms and its first execute() 66 ms (vs
     * under 5 ms and 13-20 ms on a second enable without restarting) - most of the ~0.45s stall.
     * With this warmup, the same first enable measured 7 ms and 3-5 ms.
     *
     * <p>Side-effect free: the warmup path is built in code with no event triggers (so no
     * registered auto events can fire), the follower's drive output is discarded, and it is never
     * scheduled, so it never takes the drivetrain requirement. The one real-hardware call is
     * {@link CommandSwerveDrivetrain#drive} with zero speeds, to also warm the robot-centric
     * request path FollowPath drives through; this runs from the constructor, while disabled.
     * Both flip states are run because a red-alliance auto takes Path.flip(), which a
     * blue/unknown-alliance run would leave cold.
     */
    public void warmUpPathFollowing() {
        try {
            Path warmupPath = new Path(
                    new Path.Waypoint(new Pose2d(2.0, 2.0, Rotation2d.kZero)),
                    new Path.TranslationTarget(3.0, 2.5),
                    new Path.Waypoint(new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(90))));
            for (boolean flip : new boolean[] { false, true }) {
                FollowPath follower = new FollowPath.Builder(
                        m_drivetrain,
                        m_drivetrain::getPose,
                        m_drivetrain::getChassisSpeeds,
                        speeds -> {}, // discard - never drive from the warmup
                        new PIDController(5.0, 0.0, 0.0),
                        new PIDController(7.0, 0.0, 0.0),
                        new PIDController(0.0, 0.0, 0.0))
                        .withShouldFlip(() -> flip)
                        .build(warmupPath);
                follower.initialize();
                for (int i = 0; i < 5; i++) {
                    follower.execute();
                    follower.isFinished();
                }
                follower.end(true);
            }
            m_drivetrain.drive(new ChassisSpeeds());
        } catch (RuntimeException e) {
            // A warmup must never be able to take down robot startup.
            DriverStation.reportWarning("BLine path-following warmup failed: " + e, false);
        }
    }

    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    public AutoChooser getAutoChooser() {
        return autoChooser;
    }


    public CANRange getCanRange() {
        return m_canRange;
    }

    private static Path mirroredCopy(Path source) {
        Path mirrored = source.copy();
        mirrored.mirror();
        return mirrored;
    }

    // ============= AUTO BUILDER HELPERS =============

    /**
     * Constructs an autonomous routine with a starting pose and a sequence of
     * commands.
     * 
     * @param startPose the initial robot pose for odometry reset
     * @param builder   a {@link Consumer} that accepts a
     *                  {@link SequentialCommandGroup}
     *                  to which commands should be added in the desired order
     * @return a {@link Command} that resets odometry and executes the command
     *         sequence
     */
    private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        // reset odometry
        group.addCommands(Commands.runOnce(() -> m_drivetrain.resetPose(startPose), m_drivetrain));
        builder.accept(group);
        return group;
    }

    /**
     * Constructs an autonomous routine with a sequence of commands (no pose reset).
     * 
     * @param builder a {@link Consumer} that accepts a
     *                {@link SequentialCommandGroup}
     *                to which commands should be added in the desired order
     * @return a {@link Command} that executes the command sequence
     */
    private Command auto(Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        builder.accept(group);
        return group;
    }
}
