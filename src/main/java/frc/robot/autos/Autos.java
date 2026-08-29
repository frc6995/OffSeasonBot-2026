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
import frc.robot.util.AllianceFlipUtil;

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

    private final Path workshopTest1 = new Path("workshop-test-1".toLowerCase());

    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());

    private final Supplier<Pose2d> TRENCH_START_LEFT = AllianceFlipUtil.flipped(Depot1Path.getStartPose());

    private final Path Depot2Path = new Path("depot-2".toLowerCase());

    private final Path Depot3Path = new Path("depot-3".toLowerCase());

    private final Path LeftBump1Path = new Path("path-1".toLowerCase());

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
                .withDefaultShouldFlip(); // Auto-flip trajectory when on red alliance

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============

    /**
     * Registers all autonomous routines with the auto chooser.
     */
    private void registerAutos() {

        autos.put("BLINE Depot Auto",
                () -> auto(TRENCH_START_LEFT.get(), c -> {
                    // Build BLine path commands from trajectory definitions
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


        autos.put("Bline_Workshop_test1",
                () -> auto(c -> {
                    Command workshopTest1Auto = pathBuilder.build(workshopTest1);
                    c.addCommands(workshopTest1Auto);
                }));

        // Register all autos with the chooser for driver station selection
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

    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    public AutoChooser getAutoChooser() {
        return autoChooser;
    }

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
