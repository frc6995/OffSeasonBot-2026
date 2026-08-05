package frc.robot.autos;

import static edu.wpi.first.units.Units.Meters;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
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

    private final Path workshopTest1 = new Path("workshop-test-1");
    private final Path workshopTest2 = new Path("workshop-test-2");
    private final Path Depot1Path = new Path("Left-Center-Line-Depot");
    private final Path Depot2Path = new Path("Depot-2");
    private final Path Depot3Path = new Path("Depot-3");

    private final CANRange m_canRange = new CANRange();

    // Delay before CANRange readings are trusted, so the sensor can't trip a path early
    private static final double kCANRangeDelaySeconds = 2.5;

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
        // .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start pose

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

                    c.addCommands(untilCloseToWall(Depot1, 5));

                    c.addCommands(untilCloseToWall(Depot2, 7));

                    c.addCommands(Depot3);

                }));

        autos.put("Bline_Workshop_test1",
                () -> auto(c -> {
                    Command workshopTest1Auto = pathBuilder.build(workshopTest1);

                    c.addCommands(workshopTest1Auto);
                }));
        autos.put("Bline_Workshop_test2",
                () -> auto(c -> {
                    Command workshopTest2Auto = pathBuilder.build(workshopTest2);

                    c.addCommands(workshopTest2Auto);
                }));

        autos.forEach(autoChooser::addCmd);
    }

    // Runs path until timeoutSeconds elapses or CANRange reports close-to-wall,
    // ignoring the sensor for the first specified seconds of the path.
    private Command untilCloseToWall(Command path, double timeoutSeconds) {
        Timer sensorDelayTimer = new Timer();

        return Commands.sequence(
                Commands.runOnce(sensorDelayTimer::restart),
                path.withTimeout(timeoutSeconds)
                        .until(() -> sensorDelayTimer.hasElapsed(kCANRangeDelaySeconds)
                                && m_canRange.isCloseToWall()));
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
