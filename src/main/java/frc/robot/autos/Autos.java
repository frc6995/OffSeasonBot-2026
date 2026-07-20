package frc.robot.autos;

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

public class Autos {
    private static final Pose2d kAutoAlignTestStartPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    private static final Pose2d kAutoAlignTestTargetPose = new Pose2d(4.0, 0.0, Rotation2d.kZero);
    private static final Rotation2d kAutoAlignTestEntryAngle = Rotation2d.k180deg;

    private final CommandSwerveDrivetrain drivetrain;
    private final AutoChooser autoChooser = new AutoChooser();
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final FollowPath.Builder pathBuilder;

    // ============= BLINE PATHS =============

    private final Path directionTestPath = new Path("Direction_test");
    private final Path newPath1Path = new Path("new-path1");

    public Autos(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        pathBuilder = new FollowPath.Builder(
                drivetrain,                     // Subsystem requirement
                drivetrain::getPose,            // Supplier<Pose2d>
                drivetrain::getChassisSpeeds,   // Supplier<ChassisSpeeds> (robot-relative)
                drivetrain::drive,              // Consumer<ChassisSpeeds>  (robot-relative)
                new PIDController(5.0, 0.0, 0.0),   // translation — minimizes remaining distance
                new PIDController(7.0, 0.0, 0.0),   // rotation    — minimizes heading error
                new PIDController(0.5, 0.0, 0.0)    // cross-track — minimizes perpendicular deviation
        )
                .withDefaultShouldFlip()                // auto-flip when on the red alliance
                .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start pose

        registerAutos();
    }

    // ============= AUTO REGISTRATION =============

    private void registerAutos() {
        autos.put("Test AutoAlign Distance Cancel",
                () -> auto(kAutoAlignTestStartPose, c -> {
                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(
                            AutoAlign.kDefaultVelocityLimitedProfile,
                            kAutoAlignTestTargetPose,
                            drivetrain,
                            Meters.of(0.05)));
                }));

        autos.put("Test AutoAlign Entry Angle Distance Cancel",
                () -> auto(kAutoAlignTestStartPose, c -> {
                    c.addCommands(AutoAlign.toPoseUntilWithinDistance(
                            AutoAlign.kDefaultVelocityLimitedProfile,
                            kAutoAlignTestTargetPose,
                            kAutoAlignTestEntryAngle,
                            drivetrain,
                            Meters.of(0.05)));
                }));

        autos.put("Direction_test",
                () -> auto(c -> {
                    Command directionTestAuto = pathBuilder.build(directionTestPath);

                    c.addCommands(directionTestAuto);
                }));

        autos.put("new-path1",
                () -> auto(c -> {
                    Command newPath1Auto = pathBuilder.build(newPath1Path);

                    c.addCommands(newPath1Auto);
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
