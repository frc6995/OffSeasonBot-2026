// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.power.PowerMonitor;
import frc.robot.subsystems.vision.ATVision;
import frc.robot.subsystems.vision.apriltag.NoneATLimelightVision;
import frc.robot.subsystems.vision.apriltag.RealATLimelightVision;
import frc.robot.subsystems.vision.photon.RealPhotonATVision;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlignFixedHeading;
import frc.robot.util.Elastic;
import frc.robot.util.Telemetry;
import frc.robot.util.AutoAlign.RotationControlMode;
import frc.robot.subsystems.dyerotor.DyeRotor.DyeRotorState;
import frc.robot.util.currentlimit.CurrentLimitManager;


import java.util.Set;

// @Logged
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry();
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

    public Superstructure m_superstructure = new Superstructure(m_drivetrain::state);

    // No vision simulation -- real-life testing on hardware is more useful than simulating the
    // Limelight, so simulation just runs without vision measurements at all.
    public final ATVision m_vision = new ATVision(
            Utils.isSimulation()
                ? new NoneATLimelightVision()
                : new RealATLimelightVision(NetworkTableInstance.getDefault().getTable(ATVision.ATVisionConstants.NT_TABLE)),
            true //Utils.isSimulation()
                ? null
                : new RealPhotonATVision(NetworkTableInstance.getDefault().getTable(ATVision.ATVisionConstants.NT_TABLE)),
            m_drivetrain::state,
            m_drivetrain.getPigeon2()::getRotation3d,
            m_drivetrain::addVisionMeasurement,
            m_superstructure.m_turret::getAngle);

    private Mechanism2d VISUALIZER;
    public final Autos autos = new Autos(m_drivetrain, m_superstructure);

    public final CurrentLimitManager currentLimitManager = new CurrentLimitManager();

   // public final PowerMonitor m_power = new PowerMonitor();

    public RobotContainer() {
        VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
        SmartDashboard.putData("Visualizer", VISUALIZER);
        SmartDashboard.putData("Auto Mode", autos.getAutoChooser());
        SmartDashboard.putString("Superstructure state", m_superstructure.getRobotState().toString());

        RobotCurrentLimits.configure(currentLimitManager, m_superstructure, m_drivetrain);

        configureBindings();
        SignalLogger.enableAutoLogging(false);
        RobotVisualizer.setupVisualizer();
        warmUpAutoAlignCommands();
        warmUpElastic();
    }

    /**
     * Touches {@link Elastic} here so its static initializer - including the Jackson
     * {@code ObjectMapper} it constructs for {@code sendNotification}, unrelated to
     * {@code selectTab} but initialized together as one class - runs during construction rather
     * than at the first real {@code selectTab} call. Confirmed on-robot 2026-09-19 via a scoped
     * Tracer bracketing every line of {@code teleopInit()}: {@code cancelAuto} and
     * {@code currentLimitManager.setEnabled} cost microseconds each, but {@code Elastic
     * .selectTab("Teleoperated")} alone cost 3.15-3.79s, reproducible across every boot where
     * auto never ran first (so this was the first time anything touched {@code Elastic} at all) -
     * this is what the runbook's "large overrun at start of teleop" reports were.
     */
    private void warmUpElastic() {
        Elastic.selectTab("Warmup");
    }

    /**
     * Constructs one throwaway instance of each {@code Commands.defer(...)}-wrapped auto-align
     * command bound below, purely to pay their first-use class-loading/JIT cost here during
     * construction instead of at the driver's first button press. Confirmed on-robot
     * 2026-09-19: pressing joystick.b()/x() for the first time each boot cost 115-135ms in
     * {@code DeferredCommand.initialize()} - reproducible on two separate deploys, i.e. a real,
     * repeatable one-time tax rather than random jitter, and one big enough to blow the loop
     * budget by itself. Neither constructor below does anything beyond field assignment (no CAN
     * writes, no scheduling), so building-and-discarding one of each here is side-effect-free.
     */
    private void warmUpAutoAlignCommands() {
        new AutoAlignFixedHeading(
                m_drivetrain.getPose(),
                m_drivetrain,
                true,
                RotationControlMode.VELOCITY_LIMITED_PROFILE);
        new AutoAlign(autos.TRENCH_START_LEFT.get(), m_drivetrain, AutoAlign.slowCrawlProfile());
    }

    private void configureBindings() {
        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        ));
        
        m_drivetrain.registerTelemetry(logger::telemeterize);

        /* 
        *
        *
        *   ACTUAL BINDINGS BELOW 
        *
        *
        */

        joystick.a().onTrue(m_superstructure.requestIntakeToggle());

        joystick.leftTrigger().onTrue(m_superstructure.requestIntakeEject());
       // joystick.leftTrigger().onFalse(m_superstructure.requestIntakeActive());

        // Right bumper = shoot only (flywheel/hood/turret + dye rotor); left bumper = intake
        // only. Holding both drives the intake's agitation, but only while scoring -- passing
        // shots keep the intake at plain ACTIVE. Each combination below is bound as its own
        // onTrue edge so the final intake state is always set fresh by whichever exclusive
        // trigger just became true, regardless of the order the two bumpers were pressed in.
        Trigger shootButton = joystick.rightBumper();
        Trigger intakeButton = joystick.leftBumper();
        Trigger shootAndIntake = shootButton.and(intakeButton);
        Trigger shootOnly = shootButton.and(intakeButton.negate());
        Trigger intakeOnly = intakeButton.and(shootButton.negate());

        shootButton.onTrue(m_superstructure.requestRobotShooting());
        intakeOnly.onTrue(m_superstructure.requestIntakeActive());
        intakeButton.onFalse(m_superstructure.requestIntakeIdle());
        shootOnly.onTrue(m_superstructure.requestIntakeFullAgitate());
        shootAndIntake.onTrue(m_superstructure.requestIntakeForShootAndIntake());

        // Once neither bumper is held, the intake settles back to its default ACTIVE state.
        shootButton.or(intakeButton).onFalse(m_superstructure.requestIntakeActive());

        joystick.start().and(RobotModeTriggers.disabled()).onTrue(m_superstructure.requestHomeMechanisms());

        //Safe Shot
        joystick.y().onTrue(
            m_superstructure.requestRobotShootSafe());

        // shootButton (normal shoot) and y (safe shot) both drive the shared shooting state.
        // Only return to idle once BOTH are released -- separate onFalse handlers here would
        // let releasing one button cancel a shot still being held via the other.
        shootButton.or(joystick.y()).onFalse(m_superstructure.requestRobotIdle());

        // Snap the robot's heading to the nearest cardinal direction in place.
        joystick.b().whileTrue(Commands.defer(
                () -> new AutoAlignFixedHeading(
                        m_drivetrain.getPose(),
                        m_drivetrain,
                        true,
                        RotationControlMode.VELOCITY_LIMITED_PROFILE),
                Set.of(m_drivetrain)));
        
        joystick.x().whileTrue(Commands.defer(
                () -> new AutoAlign(autos.TRENCH_START_LEFT.get(), m_drivetrain, AutoAlign.slowCrawlProfile()),
                Set.of(m_drivetrain)));

    }
  
    public Command getAutonomousCommand() {
        return autos.selectedCommand();
    }
}
