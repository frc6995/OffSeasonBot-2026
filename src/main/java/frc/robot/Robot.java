// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Elastic;
import frc.robot.util.LoopTiming;


//Don't edit this one, edit the one at line 60
@Logged(name="Robot", importance = Importance.CRITICAL)
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    @NotLogged
    private double autoSimTime = 20.0; // seconds to wait before disabling autonomous in simulation

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    //         .withTimestampReplay()
    //         .withJoystickReplay();

    public Robot() {
        // Must run before RobotContainer constructs any CTRE device. Phoenix starts hoot
        // auto-logging (on by default with a CANivore present) as soon as its backend comes up, so
        // disabling it from the end of the RobotContainer constructor - where this used to live -
        // was too late: on-robot 2026-10-03 it was still writing ~45 MB of .hoot per 6 minutes
        // (10.2 GB accumulated in /home/lvuser/logs) on a roboRIO already ~75% CPU-busy while
        // disabled. stop() covers anything that started regardless.
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();

        m_robotContainer = new RobotContainer();

        Epilogue.configure(config -> {

            if (isSimulation()) {
                // If running in simulation, then we'd want to re-throw any errors that
                // occur so we can debug and fix them!
                config.errorHandler = ErrorHandler.crashOnError();
            }

            // Power data does not come from Epilogue - it is in the hoot log (see HootLogging) -
            // so this floor has no effect on tools/power_analysis.
            config.minimumImportance = Logged.Importance.CRITICAL;
            // Only write a value to the backend when it actually changes, to save
            // bandwidth/log file size.
            config.backend = config.backend.lazy();
        });
        // DriverStation.startDataLog(DataLogManager.getLog());
       // Epilogue.bind(this);
    }

    @Override
    public void robotPeriodic() {
        LoopTiming.getInstance().start();
        CommandScheduler.getInstance().run();
        m_robotContainer.hootLogging.periodic();

        LoopTiming.getInstance().end();
       // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // Shooting-based throttling is off for auto; the drive nominal still switches to its auto limit
        m_robotContainer.currentLimitManager.setEnabled(false);

        if (RobotBase.isSimulation()) {
            CommandScheduler.getInstance().schedule(
                    Commands.waitSeconds(autoSimTime)
                            .andThen(
                                    () -> {
                                        DriverStationSim.setEnabled(false);
                                        DriverStationSim.notifyNewData();
                                    })
                            .onlyWhile(DriverStation::isAutonomousEnabled));
        }
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        //Tab switching so when we start, tab switches to "Autonomous".
        Elastic.selectTab("Autonomous");
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        // enable dynamic current limiting but only for teleop
        m_robotContainer.currentLimitManager.setEnabled(true);

        //Tab switches to "Teleoperated"
        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        // Runs after robotPeriodic(), so every subsystem's simulationPeriodic() has already pushed
        // this loop's component poses. One publish per loop instead of one per subsystem.
        RobotVisualizer.publish();
    }
}
