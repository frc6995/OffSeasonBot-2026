package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Hardware;
import frc.robot.Robot;
import frc.robot.util.POI;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.dyerotor.DyeRotorIOSimTalonFX;
import frc.robot.subsystems.dyerotor.DyeRotorIONone;
import frc.robot.subsystems.dyerotor.DyeRotorIOTalonFX;
import frc.robot.subsystems.dyerotor.DyeRotor.DyeRotorState;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSimTalonFX;
import frc.robot.subsystems.flywheel.FlywheelIONone;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIONone;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.subsystems.hood.HoodIOSimTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSimTalonFX;
import frc.robot.subsystems.intake.IntakeIONone;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSimTalonFX;
import frc.robot.subsystems.turret.TurretIONone;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.util.ShotController;

public class Superstructure extends SubsystemBase {

    public enum RobotState {
        IDLE,
        PASSING,
        SCORING
    }

    public Intake m_intake;
    public Hood m_hood;
    public Flywheel m_flywheel;
    public Turret m_turret;
    public DyeRotor m_dyeRotor;

    RobotState robotState = RobotState.IDLE;

    private final Supplier<Pose2d> m_poseSupplier;

    public final ShotController m_shotController;

    public Superstructure(Supplier<SwerveDriveState> swerveState) {
        this.m_poseSupplier = () -> swerveState.get().Pose;
        m_shotController = new ShotController(
            m_poseSupplier, () -> swerveState.get().Speeds, POI.HUB_CENTER, POI.PASSING_ANGLE,
            POI.PASSING_WALL_START, POI.PASSING_WALL_END);

        if (Robot.isSimulation()) {
            this.m_intake = new Intake(new IntakeIOSimTalonFX());
            this.m_hood = new Hood(new HoodIOSimTalonFX(), m_shotController::getCachedData);
            this.m_flywheel = new Flywheel(new FlywheelIOSimTalonFX(), m_shotController::getCachedData);
            this.m_turret = new Turret(new TurretIOSimTalonFX(), m_shotController::getCachedData);
            this.m_dyeRotor = new DyeRotor(new DyeRotorIOSimTalonFX());

        } else {
            // Mechanisms that aren't on the robot yet get a no-op IO rather than a TalonFX IO, so
            // no CAN device is ever constructed for them. See Constants.Hardware - declaring motors
            // that aren't on the bus costs a Phoenix error report per motor per loop.
            this.m_intake = new Intake(
                Hardware.kIntakeInstalled ? new IntakeIOTalonFX() : new IntakeIONone());
            this.m_hood = new Hood(
                Hardware.kHoodInstalled ? new HoodIOTalonFX() : new HoodIONone(),
                m_shotController::getCachedData);
            this.m_flywheel = new Flywheel(
                Hardware.kFlywheelInstalled ? new FlywheelIOTalonFX() : new FlywheelIONone(),
                m_shotController::getCachedData);
            this.m_turret = new Turret(
                Hardware.kTurretInstalled ? new TurretIOTalonFX() : new TurretIONone(),
                m_shotController::getCachedData);
            this.m_dyeRotor = new DyeRotor(
                Hardware.kDyeRotorInstalled ? new DyeRotorIOTalonFX() : new DyeRotorIONone());
        }

    }


    @Override
    public void periodic() {
       m_shotController.calculate(robotState == RobotState.PASSING);
    }

    public Command requestIntakeActive() {
        return Commands.runOnce(() -> m_intake.requestActive());
    }

    public Command requestIntakeRetracted() {
        return Commands.runOnce(() -> m_intake.requestRetract());
    }

    public Command requestIntakeAgitating() {
        return Commands.runOnce(() -> m_intake.requestAgitate());
    }

    // In actual use, Idle can mean slow roller velocity
    public Command requestIntakeIdle() {
        return Commands.runOnce(() -> m_intake.requestIdle());
    }

    public Command requestIntakeEject() {
        return Commands.runOnce(() -> m_intake.requestEject());
    }

    public Command requestIntakeToggle() {
        return Commands.runOnce(() -> {
            if(m_intake.isDeployed()) {
                m_intake.requestRetract();
            } else {
                m_intake.requestActive();
            }
        });
    }

    public Command requestFlywheelActive() {
        return Commands.runOnce(() -> m_flywheel.requestActive());
    }

    // Keeps the flywheel disabled for the given delay
    public Command requestFlywheelActiveAfterDelay(double seconds) {
        return Commands.runOnce(() -> m_flywheel.requestActiveAfterDelay(seconds));
    }

    public Command requestRobotIdle() {
        return Commands.runOnce(() -> {
            robotState = RobotState.IDLE;
            m_dyeRotor.requestIdle();
            m_turret.requestAimCentral();
            m_flywheel.requestDisable();
            m_hood.requestDisable();
        });
    }

    public Command requestRobotScoring() {
        return Commands.runOnce(() -> engageShootState(RobotState.SCORING));
    }

    public Command requestRobotPassing() {
        return Commands.runOnce(() -> engageShootState(RobotState.PASSING));
    }

    //This one automatically chooses PASSING or SCORING based on whether the robot is in the passing zone.
    public Command requestRobotShooting() {
        return Commands.runOnce(() -> {
            RobotState targetState = determineShootState();
            engageShootState(targetState);
        });
    }

    /** Chooses PASSING or SCORING based on whether the robot is in the configurable passing zone. */
    private RobotState determineShootState() {
        boolean inPassingZone = POI.PASSING_ZONE.get().contains(m_poseSupplier.get().getTranslation());
        return inPassingZone ? RobotState.PASSING : RobotState.SCORING;
    }

    private void engageShootState(RobotState state) {
        robotState = state;
        m_dyeRotor.requestSpin();
        m_turret.requestAimClosest();
        m_flywheel.requestActive();
        m_hood.requestActive();
    }

    public RobotState getRobotState() {
        return robotState;
    }
}
