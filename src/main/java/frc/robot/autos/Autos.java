package frc.robot.autos;

import static edu.wpi.first.units.Units.Meters;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AutoAlign;

public class Autos {
    private static final Pose2d kAutoAlignTestStartPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    private static final Pose2d kAutoAlignTestTargetPose = new Pose2d(4.0, 0.0, Rotation2d.kZero);

    private SendableChooser<Command> auto_chooser = new SendableChooser<>();
    private FollowPath.Builder pathBuilder;

    public Autos(CommandSwerveDrivetrain drivetrain) {
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

        File path_directory = new File("src/main/deploy/autos/paths");

        auto_chooser.addOption("Test AutoAlign Distance Cancel", Commands.sequence(
            Commands.runOnce(() -> drivetrain.resetPose(kAutoAlignTestStartPose), drivetrain),
            AutoAlign.toPoseUntilWithinDistance(
                AutoAlign.kDefaultVelocityLimitedProfile,
                kAutoAlignTestTargetPose,
                drivetrain,
                Meters.of(0.05))));
        
        String[] paths = path_directory.list();

        for (int i = 0; i < paths.length; i++) {
                int index = paths[i].indexOf(".json");

                paths[i] = paths[i].substring(0, index);
        }

        for (String path : paths) {
                Path bline_path = new Path(path);

                auto_chooser.addOption(path, pathBuilder.build(bline_path));
        }

        SmartDashboard.putData(auto_chooser);
    }

    public SendableChooser<Command> getAutoChooser() {
        return auto_chooser;
    }
}
