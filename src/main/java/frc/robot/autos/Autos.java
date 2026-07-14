package frc.robot.autos;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Autos {
   

    private SendableChooser<Command> auto_chooser = new SendableChooser<>();
    private FollowPath.Builder pathBuilder;

    public Autos(CommandSwerveDrivetrain drivetrain) {
        pathBuilder = new FollowPath.Builder(
            drivetrain,                     // Subsystem requirement
            drivetrain::getPose,            // Supplier<Pose2d>
            drivetrain::getChassisSpeeds,   // Supplier<ChassisSpeeds> (robot-relative)
            drivetrain::drive,              // Consumer<ChassisSpeeds>  (robot-relative)
            new PIDController(4.0, 0.0, 0.0),   // translation — minimizes remaining distance
            new PIDController(6.0, 0.0, 0.0),   // rotation    — minimizes heading error
            new PIDController(2, 0.0, 0.0)    // cross-track — minimizes perpendicular deviation
        )
        .withDefaultShouldFlip()                // auto-flip when on the red alliance
        .withPoseReset(drivetrain::resetPose); // reset odometry at each path's start pose

        File path_directory = new File("src/main/deploy/autos/paths");
        
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
