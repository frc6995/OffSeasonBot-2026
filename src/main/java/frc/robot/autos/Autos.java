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

public class Autos {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Superstructure m_superstructure;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;

    // ============= BLINE PATHS =============

    private final Path workshopTest1 = new Path("workshop-test-1".toLowerCase());
    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());
    private final Path Depot2Path = new Path("depot-2".toLowerCase());
    private final Path Depot3Path = new Path("depot-3".toLowerCase());

    private final Path LeftBump1Path = new Path("path-1".toLowerCase());
    private final Path LeftBump2Path = new Path("path-2".toLowerCase());

    private final CANRange m_canRange = new CANRange();

    public Autos(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.m_drivetrain = drivetrain;
        this.m_superstructure = superstructure;

        FollowPath.registerEventTrigger("intakeIdle", m_superstructure.requestIntakeIdle());
        FollowPath.registerEventTrigger("startShooter", m_superstructure.requestFlywheelActive());

        FollowPath.registerEventTrigger("startShooting", Commands.parallel(
                m_superstructure.requestRobotScoring()));

        FollowPath.registerEventTrigger("startIntakingAgain", m_superstructure.requestIntakeAgitating());

        FollowPath.registerEventTrigger("stopScoring",
                Commands.parallel(m_superstructure.requestRobotIdle(), superstructure.requestIntakeActive()));

        // Left Bump Path 1 Event Triggers
        

        FollowPath.registerEventTrigger("leftBumpStopIntake", m_superstructure.requestIntakeIdle());

        FollowPath.registerEventTrigger("leftBumpStartShooter", m_superstructure.requestFlywheelActive());

        FollowPath.registerEventTrigger("leftBumpStartShooting", m_superstructure.requestRobotScoring());

        FollowPath.registerEventTrigger("leftBumpStartAgitating", m_superstructure.requestIntakeAgitating());

        FollowPath.registerEventTrigger("leftBumpStopAgitating", m_superstructure.requestIntakeIdle());

        FollowPath.registerEventTrigger("leftBumpStopShooting", Commands.parallel(
                m_superstructure.requestRobotIdle(), m_superstructure.requestIntakeActive()));

        // Left Bump Path 2 Event Triggers
        FollowPath.registerEventTrigger("leftBumpStartIntakeAgain", m_superstructure.requestIntakeActive());

        FollowPath.registerEventTrigger("leftBumpStopIntakeAgain", m_superstructure.requestIntakeIdle());

        FollowPath.registerEventTrigger("leftBumpStartShooterAgain", m_superstructure.requestFlywheelActive());

        FollowPath.registerEventTrigger("leftBumpStartShootingAgain", m_superstructure.requestRobotScoring());

        FollowPath.registerEventTrigger("LeftBumpStartAgitatingAgain", m_superstructure.requestIntakeAgitating());


        // Bline Configurations
        pathBuilder = new FollowPath.Builder(
                m_drivetrain, // Subsystem requirement
                m_drivetrain::getPose, // Supplier<Pose2d>
                m_drivetrain::getChassisSpeeds, // Supplier<ChassisSpeeds> (robot-relative)
                m_drivetrain::drive, // Consumer<ChassisSpeeds> (robot-relative)
                new PIDController(5.0, 0.0, 0.0), // translation — minimizes remaining distance
                new PIDController(7.0, 0.0, 0.0), // rotation — minimizes heading error
                new PIDController(0.0, 0.0, 0.0) // cross-track — minimizes perpendicular deviation
        )
                .withDefaultShouldFlip(); // auto-flip when on the red alliance
        // .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start
        // pose

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============

    private void registerAutos() {

        autos.put("AP Depot Auto",
                () -> auto(POI.TRENCH_START.get(), c -> {

                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.kHighJerkProfile,
                            POI.M_1.get(), m_drivetrain, Meters.of(1.0)));

                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.kHighJerkProfile,
                            POI.M_2.get(), m_drivetrain, Meters.of(1.0)));

                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.kSlowDriveProfile,
                            POI.M_3.get(), m_drivetrain, Meters.of(1.0)));

                    c.addCommands(new AutoAlign(
                            POI.HUB_BEHIND_INTAKE.get(), m_drivetrain, AutoAlign.kSlowDriveProfile,
                            AutoAlign.AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY).withTimeout(2.0));
                }));

        autos.put("BLINE Depot Auto",
                () -> auto(POI.TRENCH_START.get(), c -> {

                    // BLine Path Commands
                    Command Depot1 = pathBuilder.build(Depot1Path);
                    Command Depot2 = pathBuilder.build(Depot2Path);
                    Command Depot3 = pathBuilder.build(Depot3Path);

                    c.addCommands(untilCloseToWallAfterEvent(Depot1, "hubSensorActivation", 5)
                            .alongWith(m_superstructure.requestIntakeActive()));

                    c.addCommands(untilCloseToWallAfterEvent(Depot2, "depotSensorActivation", 7));
                    // Now make the intake idle until we are near the depot

                    c.addCommands(Depot3);

                }));

        autos.put("Left Double Swipe Bump",
                () -> auto(POI.TRENCH_START.get(), c -> {

                    // BLine Path Commands
                    Command LeftBump1Cmd = pathBuilder.build(LeftBump1Path);
                    Command LeftBump2Cmd = pathBuilder.build(LeftBump2Path);

                    c.addCommands(LeftBump1Cmd.alongWith(m_superstructure.requestIntakeActive()));
                    c.addCommands(LeftBump2Cmd);

                }));

        autos.put("Bline_Workshop_test1",
                () -> auto(c -> {
                    Command workshopTest1Auto = pathBuilder.build(workshopTest1);

                    c.addCommands(workshopTest1Auto);
                }));

        autos.forEach(autoChooser::addCmd);
    }

    // Runs path until timeoutSeconds elapses or CANRange reports close-to-wall,
    // ignoring the sensor until the given BLine event marker has fired
    // along the path so the sensor can't trip the path early.
    private Command untilCloseToWallAfterEvent(Command path, String eventKey, double timeoutSeconds) {
        AtomicBoolean eventFired = new AtomicBoolean(false);
        FollowPath.registerEventTrigger(eventKey, () -> eventFired.set(true));

        return Commands.sequence(
                Commands.runOnce(() -> eventFired.set(false)),
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

    // ============= AUTO BUILDER =============

    private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        group.addCommands(Commands.runOnce(() -> m_drivetrain.resetPose(startPose), m_drivetrain));
        builder.accept(group);

        return group;
    }

    private Command auto(Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        builder.accept(group);

        return group;
    }
}
