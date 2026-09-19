// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
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

    public Robot() {
        m_robotContainer = new RobotContainer();

        Epilogue.configure(config -> {

            if (isSimulation()) {
                // If running in simulation, then we'd want to re-throw any errors that
                // occur so we can debug and fix them!
                config.errorHandler = ErrorHandler.crashOnError();
            }

            // DEBUG, not CRITICAL: the per-motor current and voltage getters throughout the
            // subsystems are annotated at DEBUG/INFO, so a CRITICAL floor silently dropped every
            // one of them - logs contained no current or voltage data at all, which made offline
            // brownout analysis impossible. See tools/power_analysis.
            //
            // This costs log file size, not field bandwidth: NT4 only transmits topics a client
            // has subscribed to, and DataLogManager's NT recording runs on the roboRIO itself.
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
        // Dynamic current limiting disabled for auto
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

    // TEMPORARY - bisecting the 3-6s stall seen at every first teleopInit() on-robot. New Tracer
    // instance so its first printEpochs() call fires unconditionally (its own internal 1s
    // throttle starts at time zero), instead of possibly being suppressed by rate limiting on a
    // shared one. Remove once the culprit line is found.
    private final edu.wpi.first.wpilibj.Tracer m_teleopInitTracer = new edu.wpi.first.wpilibj.Tracer();

    @Override
    public void teleopInit() {
        m_teleopInitTracer.resetTimer();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_teleopInitTracer.addEpoch("cancelAuto");

        // enable dynamic current limiting but only for teleop
        m_robotContainer.currentLimitManager.setEnabled(true);
        m_teleopInitTracer.addEpoch("currentLimitManager.setEnabled");

        //Tab switches to "Teleoperated"
        Elastic.selectTab("Teleoperated");
        m_teleopInitTracer.addEpoch("Elastic.selectTab");
        m_teleopInitTracer.printEpochs();
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
