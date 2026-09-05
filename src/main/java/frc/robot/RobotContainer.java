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
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.ATVision;
import frc.robot.subsystems.vision.apriltag.NoneATLimelightVision;
import frc.robot.subsystems.vision.apriltag.RealATLimelightVision;
import frc.robot.subsystems.vision.photon.RealPhotonATVision;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlignFixedHeading;
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
            Utils.isSimulation()
                ? null
                : new RealPhotonATVision(NetworkTableInstance.getDefault().getTable(ATVision.ATVisionConstants.NT_TABLE)),
            m_drivetrain::state,
            m_drivetrain.getPigeon2()::getRotation3d,
            m_drivetrain::addVisionMeasurement,
            m_superstructure.m_turret::getAngle);

    private Mechanism2d VISUALIZER;
    public final Autos autos = new Autos(m_drivetrain, m_superstructure);

    public final CurrentLimitManager currentLimitManager = new CurrentLimitManager();

    public RobotContainer() {
        VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
        SmartDashboard.putData("Visualizer", VISUALIZER);
        SmartDashboard.putData("Auto Mode", autos.getAutoChooser());
        SmartDashboard.putString("Superstructure state", m_superstructure.getRobotState().toString());

        RobotCurrentLimits.configure(currentLimitManager, m_superstructure, m_drivetrain);

        configureBindings();
        SignalLogger.enableAutoLogging(false);
        RobotVisualizer.setupVisualizer();
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
        joystick.leftTrigger().onFalse(m_superstructure.requestIntakeActive());

        joystick.rightBumper().whileTrue(m_superstructure.requestRobotShooting());

        joystick.leftBumper().onTrue(m_superstructure.requestIntakeAgitating());
        joystick.leftBumper().onFalse(m_superstructure.requestIntakeActive());

         joystick.start().and(RobotModeTriggers.disabled()).onTrue(m_superstructure.requestHomeMechanisms());

        //Safe Shot
        joystick.y().onTrue(
            m_superstructure.requestRobotShootSafe());

        // rightBumper (normal shoot) and y (safe shot) both drive the shared shooting state.
        // Only return to idle once BOTH are released -- separate onFalse handlers here would
        // let releasing one button cancel a shot still being held via the other.
        joystick.rightBumper().or(joystick.y()).onFalse(m_superstructure.requestRobotIdle());

        /* For Cadsim testing */
        // joystick.x().onTrue(Commands.runOnce(() -> m_Superstructure.m_turret.setAngle(90)));
        // joystick.y().onTrue(Commands.runOnce(() -> m_Superstructure.m_turret.setAngle(0)));
    
        // Snap the robot's heading to the nearest cardinal direction in place.
        joystick.b().whileTrue(Commands.defer(
                () -> new AutoAlignFixedHeading(
                        m_drivetrain.getPose(),
                        m_drivetrain,
                        true,
                        RotationControlMode.VELOCITY_LIMITED_PROFILE),
                Set.of(m_drivetrain)));
        
        // Deferred so the pose (and its alliance flip) is re-evaluated every time
        // the button is pressed
        joystick.x().whileTrue(Commands.defer(
                () -> new AutoAlign(autos.TRENCH_START_LEFT.get(), m_drivetrain, AutoAlign.slowCrawlProfile()),
                Set.of(m_drivetrain)));

    }
  
    public Command getAutonomousCommand() {
        return autos.selectedCommand();
    }
}
