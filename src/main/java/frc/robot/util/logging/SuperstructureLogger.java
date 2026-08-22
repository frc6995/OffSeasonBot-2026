package frc.robot.util.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.hood.HoodLogger;

@CustomLoggerFor(Superstructure.class)
public class SuperstructureLogger extends ClassSpecificLogger<Superstructure> {
    
    public SuperstructureLogger() {
        super(Superstructure.class);
    }

    @Override
    protected void update(EpilogueBackend backend, Superstructure object) {
        ErrorHandler errorHandler = Epilogue.getConfig().errorHandler;

        backend.log("Robot State", object.getRobotState());

        Epilogue.hoodLogger.tryUpdate(backend.getNested("Hood"), object.m_hood, errorHandler);
        Epilogue.intakeLogger.tryUpdate(backend.getNested("Intake"), object.m_intake, errorHandler);
        Epilogue.turretLogger.tryUpdate(backend.getNested("Turret"), object.m_turret, errorHandler);
        Epilogue.dyeRotorLogger.tryUpdate(backend.getNested("Dye Rotor"), object.m_dyeRotor, errorHandler);
        Epilogue.flywheelLogger.tryUpdate(backend.getNested("Flywheel"), object.m_flywheel, errorHandler);
    }
}
