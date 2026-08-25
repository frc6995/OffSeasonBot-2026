package frc.robot.util.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

@CustomLoggerFor(RobotContainer.class)
public class RobotContainerLogger extends ClassSpecificLogger<RobotContainer> {
    public RobotContainerLogger() {
        super(RobotContainer.class);
    }

    @Override
    protected void update(EpilogueBackend backend, RobotContainer object) {
        ErrorHandler errorHandler = Epilogue.getConfig().errorHandler;

        Epilogue.swerveDriveStateLogger.tryUpdate(backend.getNested("Swerve/State"), object.m_drivetrain.state(), errorHandler);
        Epilogue.superstructureLogger.tryUpdate(backend, object.m_superstructure, errorHandler);
    }
    
}
