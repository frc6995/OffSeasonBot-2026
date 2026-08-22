package frc.robot.util.logging;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@CustomLoggerFor(SwerveDriveState.class)
public class SwerveDriveStateLogger extends ClassSpecificLogger<SwerveDriveState> {
  public SwerveDriveStateLogger() {
    super(SwerveDriveState.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, SwerveDriveState object) {
    if(Epilogue.shouldLog(Importance.CRITICAL)) {
      dataLogger.log("cha/Pose", object.Pose, Pose2d.struct);
      dataLogger.log("cha/speed", object.Speeds, ChassisSpeeds.struct);
    }
    if(Epilogue.shouldLog(Importance.INFO)) {
      dataLogger.log("mod/targets", object.ModuleTargets, SwerveModuleState.struct);
      dataLogger.log("mod/states", object.ModuleStates, SwerveModuleState.struct);
      dataLogger.log("mod/positions", object.ModulePositions, SwerveModulePosition.struct);
    }
  }
}