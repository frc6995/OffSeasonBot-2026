package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.dyerotor.DyeRotor;
import frc.robot.subsystems.dyerotor.DyeRotorIOSimTalonFX;
import frc.robot.subsystems.dyerotor.DyeRotorIOTalonFX;
import frc.robot.subsystems.dyerotor.DyeRotor.DyeRotorState;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSimTalonFX;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.subsystems.hood.HoodIOSimTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSimTalonFX;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSimTalonFX;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.subsystems.intake.Intake.IntakeState;

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

    public RobotState getRobotState() {
        return robotState;
    }

    public Superstructure() {
        if (Robot.isSimulation()) {
            this.m_intake = new Intake(new IntakeIOSimTalonFX());
            this.m_hood = new Hood(new HoodIOSimTalonFX());
            this.m_flywheel = new Flywheel(new FlywheelIOSimTalonFX());
            this.m_turret = new Turret(new TurretIOSimTalonFX());
            this.m_dyeRotor = new DyeRotor(new DyeRotorIOSimTalonFX());

        } else {
            this.m_intake = new Intake(new IntakeIOTalonFX());
            this.m_hood = new Hood(new HoodIOTalonFX());
            this.m_flywheel = new Flywheel(new FlywheelIOTalonFX());
            this.m_turret = new Turret(new TurretIOTalonFX());
            this.m_dyeRotor = new DyeRotor(new DyeRotorIOTalonFX());
        }

    }

    public Command requestFuelIntaking() {
        return Commands.runOnce(() -> m_intake.setState(IntakeState.INTAKING));
    }

    public Command requestIntakeRetracted() {
        return Commands.runOnce(() -> m_intake.setState(IntakeState.RETRACTED));
    }

    public Command requestIntakeAgitating() {
        return Commands.runOnce(() -> m_intake.setState(IntakeState.AGITATING));
    }

    // In actual use, Idle can mean slow roller velocity
    public Command requestIntakeIdle() {
        return Commands.runOnce(() -> m_intake.setState(IntakeState.IDLE));
    }

    public Command requestIntakeEject() {
        return Commands.runOnce(() -> m_intake.setState(IntakeState.EJECTING));
    }

    public Command requestRobotIdle() {

        return Commands.runOnce(() -> {
            robotState = RobotState.IDLE;
            m_dyeRotor.setState(DyeRotorState.SPIN_BACKWARDS);
            m_turret.setState(TurretState.DISABLED);
            m_flywheel.setState(FlywheelState.DISABLED);
        });
    }

    public Command requestRobotShooting() {
        return Commands.runOnce(() -> {
            m_flywheel.setState(FlywheelState.ACTIVE);
        });
    }

    public Command requestRobotScoring() {

        return Commands.runOnce(() -> {
            robotState = RobotState.SCORING;
            m_dyeRotor.setState(DyeRotorState.SPIN);
            m_turret.setState(TurretState.AIM_CLOSEST);
            m_flywheel.setState(FlywheelState.ACTIVE);
        });
    }

    public Command requestRobotPassing() {
        return Commands.runOnce(() -> {
            robotState = RobotState.PASSING;
            m_flywheel.setState(FlywheelState.ACTIVE);
            m_dyeRotor.setState(DyeRotorState.SPIN);
            m_turret.setState(TurretState.AIM_CLOSEST);
        });
    }
}
