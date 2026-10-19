// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    /**
     * Turns the .wpilog on or off: DataLogManager, Driver Station data and Epilogue, which carries
     * the power data for tools/power_analysis (see PowerLogging). Change it and redeploy.
     */
    public static final boolean kLoggingEnabled = true;

    public Robot() {
        // Hoot logging stays off; power data is in the .wpilog instead (kLoggingEnabled).
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

            // PowerLogging writes regardless of this floor; it only limits the @Logged fields.
            config.minimumImportance = Logged.Importance.CRITICAL;
        });

        if (kLoggingEnabled) {
            // Only what Epilogue writes. Recording every NetworkTables topic as well (vision,
            // dashboards) would grow the file and the roboRIO's logging work; flip this to get it.
            // Also starts DataLogManager: a USB stick's logs folder if present, else
            // /home/lvuser/logs, deleting the oldest logs when space runs low.
            DataLogManager.logNetworkTables(false);
            // Enabled/autonomous for the dashboard. Joysticks off: they change every loop.
            DriverStation.startDataLog(DataLogManager.getLog(), false);

            // Straight to the log file. The default NetworkTables backend publishes every value
            // and then DataLogManager records it again, which costs far more per loop.
            // Lazy: only write a value when it changes, to save log file size.
            Epilogue.getConfig().backend = new FileBackend(DataLogManager.getLog()).lazy();
        }
    }

    @Override
    public void robotPeriodic() {
        LoopTiming.getInstance().start();

        // CommandScheduler.run() carries its own Watchdog (20 ms default) and already prints
        // a per-subsystem-periodic()/per-command-execute() epoch breakdown automatically when
        // IT overruns - see CommandScheduler.java. A second Watchdog wrapping this call used to
        // live here, but it only ever produced one coarse "CommandScheduler.run()" epoch (strictly
        // less informative than the breakdown above) and - because printEpochs()/reset() ran
        // unconditionally every loop rather than only on overrun - it also added a real, permanent
        // per-loop cost: reset()/enable() takes the Watchdog class's shared queue mutex and
        // reinserts into its static TreeSet of pending watchdogs on every single loop, and
        // printEpochs() emitted a DriverStation warning roughly once a second forever, not just
        // when something was actually wrong. That is exactly the kind of unconditional per-loop
        // work + log spam this runbook tells you to hunt for elsewhere - removed rather than left
        // as a self-inflicted false lead. See tools/loop_overrun/README.md.
        CommandScheduler.getInstance().run();
        // Inside the timed section, so LoopTiming includes what logging costs. Epilogue also
        // writes its own run time to Epilogue/Stats/Last Run.
        if (kLoggingEnabled) {
            Epilogue.update(this);
        }

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
        m_robotContainer.saveLimelightRewind();
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
