package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.POI;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.dyerotor.DyeRotorIOSimTalonFX;
import frc.robot.subsystems.dyerotor.DyeRotorIOTalonFX;
import frc.robot.subsystems.dyerotor.DyeRotor.DyeRotorState;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIONone;
import frc.robot.subsystems.flywheel.FlywheelIOSimTalonFX;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIONone;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.subsystems.hood.HoodIOSimTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIONone;
import frc.robot.subsystems.intake.IntakeIOSimTalonFX;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIONone;
import frc.robot.subsystems.turret.TurretIOSimTalonFX;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.util.ShotController;

public class Superstructure extends SubsystemBase {

    public enum RobotState {
        IDLE,
        PASSING,
        SCORING,
        SAFE_SHOT
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

        // Each mechanism picks its IO in three ways:
        //  - actually simulating (desktop sim)   -> sim IO, physics model, regardless of the flag
        //  - real robot, flag installed          -> real TalonFX-backed IO
        //  - real robot, flag NOT installed      -> None IO: touches no hardware, zero CAN traffic
        // See Constants.HardwarePresence.
        this.m_intake = Robot.isSimulation()
            ? new Intake(new IntakeIOSimTalonFX())
            : Constants.HardwarePresence.kIntakeInstalled
                ? new Intake(new IntakeIOTalonFX())
                : new Intake(new IntakeIONone());
        this.m_hood = Robot.isSimulation()
            ? new Hood(new HoodIOSimTalonFX(), m_shotController::getCachedData)
            : Constants.HardwarePresence.kHoodInstalled
                ? new Hood(new HoodIOTalonFX(), m_shotController::getCachedData)
                : new Hood(new HoodIONone(), m_shotController::getCachedData);
        this.m_flywheel = Robot.isSimulation()
            ? new Flywheel(new FlywheelIOSimTalonFX(), m_shotController::getCachedData)
            : Constants.HardwarePresence.kFlywheelInstalled
                ? new Flywheel(new FlywheelIOTalonFX(), m_shotController::getCachedData)
                : new Flywheel(new FlywheelIONone(), m_shotController::getCachedData);
        this.m_turret = Robot.isSimulation()
            ? new Turret(new TurretIOSimTalonFX(), m_shotController::getCachedData)
            : Constants.HardwarePresence.kTurretInstalled
                ? new Turret(new TurretIOTalonFX(), m_shotController::getCachedData)
                : new Turret(new TurretIONone(), m_shotController::getCachedData);
        // DyeRotor is always physically present; it only ever depends on sim vs. real.
        this.m_dyeRotor = new DyeRotor(
            Robot.isSimulation() ? new DyeRotorIOSimTalonFX() : new DyeRotorIOTalonFX());
    }


    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // robotState must not survive a disable, for the same reason the mechanism states
            // can't (see Flywheel.periodic): requestRobotIdle() is bound to the shoot button's
            // onFalse edge and to an end-of-auto marker, and neither can run while disabled.
            //
            // Unlike the mechanisms, the damage here is not a mechanism that restarts itself --
            // it is that RobotCurrentLimits throttles the drivetrain to 1A whenever this reads
            // SCORING or PASSING. An auto that ends before its stopScoring marker would carry
            // that state through the disable, and teleopInit() re-enables the limit manager, so
            // teleop would start with a near-immobile drivetrain until the driver pressed and
            // released the shoot button.
            robotState = RobotState.IDLE;
        }

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

    public Command requestRobotShootSafe(){
        return Commands.runOnce(()->{engageShootState(RobotState.SAFE_SHOT);});
    }

    /** Chooses PASSING or SCORING based on whether the robot is in the configurable passing zone. */
    private RobotState determineShootState() {
        boolean inPassingZone = POI.PASSING_ZONE.get().contains(m_poseSupplier.get().getTranslation());
        return inPassingZone ? RobotState.PASSING : RobotState.SCORING;
    }

    private void engageShootState(RobotState state) {
        switch (state) {
            case SCORING, PASSING -> {
                m_turret.requestAimClosest();
                m_flywheel.requestActive();
                m_hood.requestActive();
            }
            case SAFE_SHOT -> {
                m_turret.setState(TurretState.SAFE_SHOT);
                m_flywheel.setState(FlywheelState.SAFE_SHOT);
                m_hood.setState(HoodState.SAFE_SHOT);
            }
            
        }
        robotState = state;
        m_dyeRotor.requestSpin();
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public Command requestHomeMechanisms() {
        return Commands.runOnce(() -> {
            m_intake.resetEncoder();
            m_hood.resetEncoder();
            m_turret.resetEncoder();
        }).ignoringDisable(true);
    }
}
