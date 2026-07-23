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

    public Superstructure(Intake m_intake, Hood m_hood, Flywheel m_flywheel, Turret m_turret, DyeRotor m_dyeRotor) {
        if (Robot.isSimulation()) {
            m_intake = new Intake(new IntakeIOSimTalonFX());
            m_hood = new Hood(new HoodIOSimTalonFX());
            m_flywheel = new Flywheel(new FlywheelIOSimTalonFX());
            m_turret = new Turret(new TurretIOSimTalonFX());
            m_dyeRotor = new DyeRotor(new DyeRotorIOSimTalonFX());

        } else {
            m_intake = new Intake(new IntakeIOTalonFX());
            m_hood = new Hood(new HoodIOTalonFX());
            m_flywheel = new Flywheel(new FlywheelIOTalonFX());
            m_turret = new Turret(new TurretIOTalonFX());
            m_dyeRotor = new DyeRotor(new DyeRotorIOTalonFX());
        }

    }

    public void requestIntakeDeployed() {
        m_intake.setState(IntakeState.DEPLOYED);

    }

    public void requestIntakeRetracted() {
        m_intake.setState(IntakeState.RETRACTED);

    }

    public void requestIntakeAgitating() {
        m_intake.setState(IntakeState.AGITATING);

    }

    public void requestIntakeIdle() {
        m_intake.setState(IntakeState.IDLE);

    }

    public Command requestRobotIdle() {

        return Commands.runOnce(() -> {
            robotState = RobotState.IDLE;
            m_dyeRotor.setState(DyeRotorState.SPIN_BACKWARDS);
            m_turret.setState(TurretState.DISABLED);
        });
    }

    public Command requestRobotScoring() {

        return Commands.runOnce(() -> {
            robotState = RobotState.SCORING;
            m_dyeRotor.setState(DyeRotorState.SPIN);
            m_turret.setState(TurretState.ACTIVE);
            m_flywheel.setState(FlywheelState.ACTIVE);
        });
    }

    public Command requestRobotPassing() {
        return Commands.runOnce(() -> {
            robotState = RobotState.PASSING;
            m_flywheel.setState(FlywheelState.ACTIVE);
            m_dyeRotor.setState(DyeRotorState.SPIN);
            m_turret.setState(TurretState.ACTIVE);
        });
    }
}
