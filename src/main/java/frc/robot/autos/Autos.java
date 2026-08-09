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
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;

public class Autos {

    private final CommandSwerveDrivetrain drivetrain;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;

    // ============= BLINE PATHS =============

    private final Path workshopTest1 = new Path("workshop-test-1".toLowerCase());
    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());
    private final Path Depot2Path = new Path("depot-2".toLowerCase());
    private final Path Depot3Path = new Path("depot-3".toLowerCase());

    private final CANRange m_canRange = new CANRange();

    public Autos(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Bline Configurations
        pathBuilder = new FollowPath.Builder(
                drivetrain, // Subsystem requirement
                drivetrain::getPose, // Supplier<Pose2d>
                drivetrain::getChassisSpeeds, // Supplier<ChassisSpeeds> (robot-relative)
                drivetrain::drive, // Consumer<ChassisSpeeds> (robot-relative)
                new PIDController(5.0, 0.0, 0.0), // translation — minimizes remaining distance
                new PIDController(7.0, 0.0, 0.0), // rotation — minimizes heading error
                new PIDController(0.0, 0.0, 0.0) // cross-track — minimizes perpendicular deviation
        )
                .withDefaultShouldFlip(); // auto-flip when on the red alliance
     //    .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start pose

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============

    private void registerAutos() {

        autos.put("AP Depot Auto",
                () -> auto(POI.TRENCH_START.get(), c -> {

                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.kHighJerkProfile,
                            POI.M_1.get(), drivetrain, Meters.of(1.0)));

                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.kHighJerkProfile,
                            POI.M_2.get(), drivetrain, Meters.of(1.0)));

                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(AutoAlign.kSlowDriveProfile,
                            POI.M_3.get(), drivetrain, Meters.of(1.0)));

                    c.addCommands(new AutoAlign(
                            POI.HUB_BEHIND_INTAKE.get(), drivetrain, AutoAlign.kSlowDriveProfile,
                            AutoAlign.AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY).withTimeout(2.0));
                }));

        autos.put("BLINE Depot Auto",
                () -> auto(POI.TRENCH_START.get(), c -> {

                    // BLine Path Commands
                    Command Depot1 = pathBuilder.build(Depot1Path);
                    Command Depot2 = pathBuilder.build(Depot2Path);
                    Command Depot3 = pathBuilder.build(Depot3Path);

                    c.addCommands(untilCloseToWallAfterEvent(Depot1, "hubSensorActivation", 5));

                    c.addCommands(untilCloseToWallAfterEvent(Depot2, "depotSensorActivation", 7));
                    //Now make the intake idle until we are near the depot

                    c.addCommands(Depot3);

                }));

        autos.put("Bline_Workshop_test1",
                () -> auto(c -> {
                    Command workshopTest1Auto = pathBuilder.build(workshopTest1);

                    c.addCommands(workshopTest1Auto);
                }));

        autos.forEach(autoChooser::addCmd);
    }

    // Runs path until timeoutSeconds elapses or CANRange reports close-to-wall,
    // ignoring the sensor until the given BLine event marker (lib_key) has fired
    // along the path, so the sensor can't trip the path early.
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

        group.addCommands(Commands.runOnce(() -> drivetrain.resetPose(startPose), drivetrain));
        builder.accept(group);

        return group;
    }

    private Command auto(Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        builder.accept(group);

        return group;
    }
}
