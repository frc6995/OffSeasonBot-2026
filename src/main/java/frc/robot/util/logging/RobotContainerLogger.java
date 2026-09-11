package frc.robot.util.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import frc.robot.Constants;
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

        // Both of these log only supply-current data. Gated together so that with power logging
        // off they cannot publish a tree of stale zeros - the drivetrain getters would otherwise
        // still be CRITICAL and still be read, just never refreshed.
        if (Constants.kPowerLoggingEnabled) {
            Epilogue.commandSwerveDrivetrainLogger.tryUpdate(backend.getNested("Swerve"), object.m_drivetrain, errorHandler);
            Epilogue.powerMonitorLogger.tryUpdate(backend.getNested("Power"), object.m_power, errorHandler);
        }
    }
    
}
