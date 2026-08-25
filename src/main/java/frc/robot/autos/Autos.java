package frc.robot.autos;

import static edu.wpi.first.units.Units.Meters;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CANRange;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;

/**
 * Manages all autonomous routines for the robot.
 *
 * <p>This class provides:
 * <ul>
 *   <li>Registration and management of autonomous routines via {@link AutoChooser}
 *   <li>Path definitions using BLine trajectory following
 *   <li>Event trigger registration for subsystem commands at specific path markers
 *   <li>Utilities for building autos with pose reset and sensor-based termination
 * </ul>
 *
 * <p><strong>Autonomous Routines:</strong>
 * <ul>
 *   <li><strong>AP Depot Auto</strong> - Uses AutoAlign to navigate to multiple waypoints
 *   <li><strong>BLINE Depot Auto</strong> - BLine path-based routine with sensor-based early termination
 *   <li><strong>Left Double Swipe Bump</strong> - Two-path bumper routine with intake/idle states
 *   <li><strong>Bline_Workshop_test1</strong> - Test routine for path development
 * </ul>
 *
 * <p><strong>Path Following:</strong> Uses BLine library with PID controllers configured for:
 * <ul>
 *   <li>Translation: P=5.0 - minimizes distance error
 *   <li>Rotation: P=7.0 - minimizes heading error
 *   <li>Cross-track: P=0.0 - minimizes perpendicular deviation
 * </ul>
 */
public class Autos {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Superstructure m_superstructure;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;

    // ============= BLINE PATHS =============
    /**
     * BLine trajectory paths for autonomous routines. Each Path references a trajectory file
     * (in lowercase) that contains waypoints and event markers for the path following controller.
     */

    /** Workshop test path - for development and testing */
    private final Path workshopTest1 = new Path("workshop-test-1".toLowerCase());

    /** Depot routine - first segment from starting position to center line */
    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());

    /** Depot routine - second segment for depot ball collection */
    private final Path Depot2Path = new Path("depot-2".toLowerCase());

    /** Depot routine - third segment for hub deposit */
    private final Path Depot3Path = new Path("depot-3".toLowerCase());

    /** Left bumper routine - first swipe path */
    private final Path LeftBump1Path = new Path("path-1".toLowerCase());

    /** Left bumper routine - second swipe path */
    private final Path LeftBump2Path = new Path("path-2".toLowerCase());

    /** Range sensor for detecting proximity to field walls/depot. Used for sensor-based path termination. */
    private final CANRange m_canRange = new CANRange();

    /**
     * Creates an Autos manager with the given drivetrain and superstructure.
     *
     * <p>Initializes:
     * <ul>
     *   <li>Event triggers for path markers that command subsystem states
     *   <li>BLine path following configuration with PID controllers
     *   <li>Auto routine registration
     * </ul>
     *
     * @param drivetrain the swerve drivetrain subsystem for path following
     * @param superstructure the robot's superstructure (intake, shooter, etc.)
     */
    public Autos(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.m_drivetrain = drivetrain;
        this.m_superstructure = superstructure;

        // ===== DEPOT AUTO EVENT TRIGGERS =====
        FollowPath.registerEventTrigger("intakeIdle", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("startShooter", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("startShooting", Commands.parallel(
                m_superstructure.requestRobotScoring()));
        FollowPath.registerEventTrigger("startIntakingAgain", m_superstructure.requestIntakeAgitating());
        FollowPath.registerEventTrigger("stopScoring",
                Commands.parallel(m_superstructure.requestRobotIdle(), superstructure.requestIntakeActive()));

        // ===== LEFT BUMP PATH 1 EVENT TRIGGERS =====
        FollowPath.registerEventTrigger("leftBumpStopIntake", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStartShooter", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("leftBumpStartShooting", m_superstructure.requestRobotScoring());
        FollowPath.registerEventTrigger("leftBumpStartAgitating", m_superstructure.requestIntakeAgitating());
        FollowPath.registerEventTrigger("leftBumpStopAgitating", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStopShooting", m_superstructure.requestRobotIdle());

        // ===== LEFT BUMP PATH 2 EVENT TRIGGERS =====
        FollowPath.registerEventTrigger("leftBumpStopIntakeAgain", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("leftBumpStartShooterAgain", m_superstructure.requestFlywheelActive());
        FollowPath.registerEventTrigger("leftBumpStartShootingAgain", m_superstructure.requestRobotScoring());
        FollowPath.registerEventTrigger("leftBumpStartAgitatingAgain", m_superstructure.requestIntakeAgitating());

        // ===== BLINE PATH FOLLOWING CONFIGURATION =====
        pathBuilder = new FollowPath.Builder(
                m_drivetrain, // Subsystem requirement
                m_drivetrain::getPose, // Supplier<Pose2d> - current robot pose from odometry
                m_drivetrain::getChassisSpeeds, // Supplier<ChassisSpeeds> (robot-relative) - current velocities
                m_drivetrain::drive, // Consumer<ChassisSpeeds> (robot-relative) - drivetrain command handler
                new PIDController(5.0, 0.0, 0.0), // Translation PID - minimizes remaining distance error
                new PIDController(7.0, 0.0, 0.0), // Rotation PID - minimizes heading error
                new PIDController(0.0, 0.0, 0.0) // Cross-track PID - minimizes perpendicular deviation
        )
                .withDefaultShouldFlip(); // Auto-flip trajectory when on red alliance (FRC field symmetry)

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============

    /**
     * Registers all autonomous routines with the auto chooser.
     *
     * <p>Routines include:
     * <ul>
     *   <li><strong>AP Depot Auto</strong> - MultiPoint navigation using AutoAlign kinematic calculations
     *   <li><strong>BLINE Depot Auto</strong> - Trajectory-based depot routine with sensor early termination
     *   <li><strong>Left Double Swipe Bump</strong> - Two consecutive bumper-side paths with intake management
     *   <li><strong>Bline_Workshop_test1</strong> - Single-path test routine for development
     * </ul>
     */
    private void registerAutos() {

        // ===== AP DEPOT AUTO =====
        // Uses kinematic path following to navigate multiple waypoints on the field.
        // Start at TRENCH_START, move through M_1, M_2, M_3, then final alignment at HUB_BEHIND_INTAKE.
        // AutoAlign uses motion profiles for smooth, predictable motion.
        autos.put("AP Depot Auto",
                () -> auto(POI.TRENCH_START.get(), c -> {
                    // Navigate to checkpoint M_1 with fast motion profile, stopping within 1m
                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.highJerkProfile(),
                            POI.M_1.get(), m_drivetrain, Meters.of(1.0)));

                    // Navigate to checkpoint M_2 with fast motion profile, stopping within 1m
                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.highJerkProfile(),
                            POI.M_2.get(), m_drivetrain, Meters.of(1.0)));

                    // Navigate to checkpoint M_3 with slower, more controlled motion profile, stopping within 1m
                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.slowDriveProfile(),
                            POI.M_3.get(), m_drivetrain, Meters.of(1.0)));

                    // Final alignment at hub from behind intake position with 2s timeout
                    c.addCommands(new AutoAlign(
                            POI.HUB_BEHIND_INTAKE.get(), m_drivetrain, AutoAlign.slowDriveProfile(),
                            AutoAlign.AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY).withTimeout(2.0));
                }));

        // ===== BLINE DEPOT AUTO =====
        // Uses pre-generated BLine trajectories for faster, more predictable motion.
        // Depot1: Start intake, move to hub area - terminates when sensor detects hub or timeout (5s)
        // Depot2: Move to depot - terminates when sensor detects wall/depot or timeout (7s)
        // Depot3: Return to hub for scoring
        autos.put("BLINE Depot Auto",
                () -> auto(POI.TRENCH_START.get(), c -> {
                    // Build BLine path commands from trajectory definitions
                    Command Depot1 = pathBuilder.build(Depot1Path);
                    Command Depot2 = pathBuilder.build(Depot2Path);
                    Command Depot3 = pathBuilder.build(Depot3Path);

                    // Segment 1: Move to hub area with active intake, stop early if sensor detects hub
                    c.addCommands(untilCloseToWallAfterEvent(Depot1, "hubSensorActivation", 5)
                            .alongWith(m_superstructure.requestIntakeActive()));

                    // Segment 2: Move to depot, stop early if sensor detects wall
                    c.addCommands(untilCloseToWallAfterEvent(Depot2, "depotSensorActivation", 7));

                    // Segment 3: Return to hub for final scoring
                    c.addCommands(Depot3);
                }));

        // ===== LEFT DOUBLE SWIPE BUMP =====
        // Two consecutive swipe paths starting from left bumper position.
        // Path 1: Initial swipe with intake active and robot idle state
        // Path 2: Second swipe with intake active (collects additional balls on field)
        autos.put("Left Double Swipe Bump",
                () -> auto(POI.TRENCH_START.get(), c -> {
                    // Build BLine path commands
                    Command LeftBump1Cmd = pathBuilder.build(LeftBump1Path);
                    Command LeftBump2Cmd = pathBuilder.build(LeftBump2Path);

                    // First swipe path: intake active, robot idle (preparing for collection)
                    c.addCommands(LeftBump1Cmd.alongWith(m_superstructure.requestIntakeActive(),
                            m_superstructure.requestRobotIdle()));

                    // Second swipe path: intake active to collect any balls on field
                    c.addCommands(LeftBump2Cmd.alongWith(m_superstructure.requestIntakeActive()));
                }));

        // ===== BLINE WORKSHOP TEST 1 =====
        // Single-path test routine for validating BLine path following during development
        autos.put("Bline_Workshop_test1",
                () -> auto(c -> {
                    Command workshopTest1Auto = pathBuilder.build(workshopTest1);
                    c.addCommands(workshopTest1Auto);
                }));

        // Register all autos with the chooser for driver selection
        autos.forEach(autoChooser::addCmd);
    }

    /**
     * Runs a path command until it times out or the robot is close to a wall/obstacle,
     * but only after a specified event marker has fired in the trajectory.
     *
     * <p>This prevents the sensor from triggering early by requiring that a path event
     * (e.g., "hubSensorActivation") has fired before the CANRange sensor can terminate the path.
     * Useful for depot/hub detection where you want to ensure the robot has reached
     * approximately the right location before checking distance.
     *
     * @param path the BLine path command to follow
     * @param eventKey the path event marker name that must fire before sensor termination is active
     * @param timeoutSeconds maximum time to allow the path to run (safety limit)
     * @return a command that follows the path with sensor-based early termination
     */
    private Command untilCloseToWallAfterEvent(Command path, String eventKey, double timeoutSeconds) {
        // Flag tracks whether the event has fired
        AtomicBoolean eventFired = new AtomicBoolean(false);
        // Register this trigger to set the flag when the event fires
        FollowPath.registerEventTrigger(eventKey, () -> eventFired.set(true));

        return Commands.sequence(
                // Reset flag at start of command
                Commands.runOnce(() -> eventFired.set(false)),
                // Run path with timeout and terminate when event + sensor both trigger
                path.withTimeout(timeoutSeconds)
                        .until(() -> eventFired.get() && m_canRange.isCloseToWall()));
    }

    /**
     * Gets the currently selected autonomous command from the driver's chooser selection.
     *
     * @return the command for the selected autonomous routine
     */
    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    /**
     * Gets the auto chooser for dashboard integration.
     *
     * @return the {@link AutoChooser} containing all registered autonomous routines
     */
    public AutoChooser getAutoChooser() {
        return autoChooser;
    }

    /**
     * Gets the CANRange sensor used for proximity detection during autonomous.
     *
     * @return the range sensor for wall/obstacle detection
     */
    public CANRange getCanRange() {
        return m_canRange;
    }

    // ============= AUTO BUILDER HELPERS =============

    /**
     * Constructs an autonomous routine with a known starting pose and a sequence of commands.
     *
     * <p>This variant is used when the robot's starting position is known and odometry needs
     * to be reset to that pose. The routine will:
     * <ol>
     *   <li>Reset the drivetrain odometry to the specified starting pose
     *   <li>Execute all commands added to the builder in sequence
     * </ol>
     *
     * @param startPose the initial robot pose for odometry reset (from POI definitions)
     * @param builder a {@link Consumer} that accepts a {@link SequentialCommandGroup}
     *                to which commands should be added in the desired order
     * @return a {@link Command} that resets odometry and executes the built command sequence
     */
    private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        // Reset odometry to the starting pose first
        group.addCommands(Commands.runOnce(() -> m_drivetrain.resetPose(startPose), m_drivetrain));
        // Build the rest of the autonomous routine
        builder.accept(group);

        return group;
    }

    /**
     * Constructs an autonomous routine with a sequence of commands (no pose reset).
     *
     * <p>This variant is used when odometry reset is not needed, or when it's handled
     * elsewhere. The routine will execute all commands added to the builder in sequence.
     *
     * @param builder a {@link Consumer} that accepts a {@link SequentialCommandGroup}
     *                to which commands should be added in the desired order
     * @return a {@link Command} that executes the built command sequence
     */
    private Command auto(Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        builder.accept(group);

        return group;
    }
}
