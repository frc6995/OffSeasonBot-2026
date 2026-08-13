// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.AutoAlignFixedHeading;
import frc.robot.util.Telemetry;
import frc.robot.util.AutoAlign.RotationControlMode;

import java.util.Set;

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

    public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    public Superstructure m_Superstructure = new Superstructure(drivetrain::getState);
    private Mechanism2d VISUALIZER;
    public final Autos autos = new Autos(drivetrain);

    public RobotContainer() {
        VISUALIZER = logger.MECH_VISUALIZER;
        SmartDashboard.putData("Visualizer", VISUALIZER);
        SmartDashboard.putData("Auto Mode", autos.getAutoChooser());

        configureBindings();
        SignalLogger.enableAutoLogging(false);
        RobotVisualizer.setupVisualizer();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        ));
        
        drivetrain.registerTelemetry(logger::telemeterize);

        /* 
        *
        *
        *   ACTUAL BINDINGS BELOW 
        *
        *
        */

        joystick.a().onTrue(m_Superstructure.requestIntakeToggle());

        joystick.leftTrigger().onTrue(m_Superstructure.requestIntakeEject());
        joystick.leftTrigger().onFalse(m_Superstructure.requestIntakeActive());

        joystick.rightBumper().whileTrue(m_Superstructure.requestRobotShooting());
        joystick.rightBumper().onFalse(m_Superstructure.requestRobotIdle());

        joystick.leftBumper().onTrue(m_Superstructure.requestIntakeAgitating());
        joystick.leftBumper().onFalse(m_Superstructure.requestIntakeActive());

        joystick.x().onTrue(m_Superstructure.requestExampleExtend());
        joystick.y().onTrue(m_Superstructure.requestExampleRetract());

        /* For Cadsim testing */
        // joystick.x().onTrue(Commands.runOnce(() -> m_Superstructure.m_turret.setAngle(90)));
        // joystick.y().onTrue(Commands.runOnce(() -> m_Superstructure.m_turret.setAngle(0)));
    
        // Snap the robot's heading to the nearest cardinal direction in place.
        joystick.b().whileTrue(Commands.defer(
                () -> new AutoAlignFixedHeading(
                        drivetrain.getPose(),
                        drivetrain,
                        true,
                        RotationControlMode.VELOCITY_LIMITED_PROFILE),
                Set.of(drivetrain)));


        /* 
        *
        *
        *   TEST BINDINGS BELOW 
        *
        *
        */

        // joystick.rightStick().onTrue(Commands.runOnce(() -> m_Superstucture.m_turret.setAngle(30)));

        // joystick.a().onTrue(Commands.runOnce(() -> m_Superstructure.m_dyeRotor.requestSpin()));
        // joystick.b().onTrue(Commands.runOnce(() -> m_Superstructure.m_dyeRotor.requestIdle()));

        // Sim-only: hold right bumper to fake the CANRange detecting the wall so BLine Depot Auto's
        // path transitions can be tested. No effect on real hardware (see CANRange.isCloseToWall).
        autos.getCanRange().setSimProximitySupplier(joystick.x());
    }
  
    public Command getAutonomousCommand() {
        return autos.selectedCommand();
    }
}