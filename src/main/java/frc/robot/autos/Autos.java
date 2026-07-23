package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlignFixedHeading;

public class Autos {
    // Just for testing AutoAlign
    private static final Pose2d kAutoAlignTestStartPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    private static final Pose2d kAutoAlignTestTargetPose = new Pose2d(4.0, 0.0, Rotation2d.kZero);
    private static final Pose2d kAutoAlignProfiledRotationTestTargetPose = new Pose2d(11.0, 0.0, new Rotation2d(Degrees.of(70)));
    private static final Rotation2d kAutoAlignFixedHeadingRotationTestHeading = Rotation2d.fromDegrees(90);

    private final CommandSwerveDrivetrain drivetrain;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;

    // ============= BLINE PATHS =============

    private final Path directionTestPath = new Path("Direction_test");
    private final Path workshopTest1 = new Path("workshop-test-1");
    private final Path workshopTest2 = new Path("workshop-test-2");

    public Autos(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        pathBuilder = new FollowPath.Builder(
                drivetrain, // Subsystem requirement
                drivetrain::getPose, // Supplier<Pose2d>
                drivetrain::getChassisSpeeds, // Supplier<ChassisSpeeds> (robot-relative)
                drivetrain::drive, // Consumer<ChassisSpeeds> (robot-relative)
                new PIDController(5.0, 0.0, 0.0), // translation — minimizes remaining distance
                new PIDController(7.0, 0.0, 0.0), // rotation — minimizes heading error
                new PIDController(0.5, 0.0, 0.0) // cross-track — minimizes perpendicular deviation
        )
                .withDefaultShouldFlip() // auto-flip when on the red alliance
                .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start pose

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============

    private void registerAutos() {
        autos.put("Test AutoAlign Distance Cancel",
                // In actual use, this pose will need to be flipped
                () -> auto(kAutoAlignTestStartPose, c -> {
                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(
                            AutoAlign.kDefaultVelocityLimitedProfile,
                            kAutoAlignTestTargetPose,
                            drivetrain,
                            Meters.of(0.05),
                            AutoAlign.RotationControlMode.UNPROFILED_PID,
                            AutoAlign.AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY));
                }));

        autos.put("Test AutoAlign Profiled Rotation",
                () -> auto(kAutoAlignTestStartPose, c -> {
                    c.addCommands(new AutoAlign(
                            kAutoAlignProfiledRotationTestTargetPose,
                            drivetrain,
                            AutoAlign.kDefaultVelocityLimitedProfile,
                            AutoAlign.RotationControlMode.VELOCITY_LIMITED_PROFILE,
                            AutoAlign.AutoAlignConstants.PROFILED_ROTATION_SLOW_VELOCITY));
                }));

        autos.put("Test AutoAlign Fixed Heading 90",
                () -> auto(kAutoAlignTestStartPose, c -> {
                    c.addCommands(new AutoAlignFixedHeading(
                            kAutoAlignTestStartPose,
                            drivetrain,
                            kAutoAlignFixedHeadingRotationTestHeading,
                            AutoAlign.RotationControlMode.VELOCITY_LIMITED_PROFILE));
                }));

        autos.put("BLINE_test",
                () -> auto(c -> {
                    Command directionTestAuto = pathBuilder.build(directionTestPath);

                    c.addCommands(directionTestAuto);
                }));

        autos.put("Workshop_test1",
                () -> auto(c -> {
                    Command workshopTest1Auto = pathBuilder.build(workshopTest1);

                    c.addCommands(workshopTest1Auto);
                }));
        autos.put("Workshop_test2",
                () -> auto(c -> {
                    Command workshopTest2Auto = pathBuilder.build(workshopTest2);

                    c.addCommands(workshopTest2Auto);
                }));

        autos.forEach(autoChooser::addCmd);
    }

    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    public AutoChooser getAutoChooser() {
        return autoChooser;
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
