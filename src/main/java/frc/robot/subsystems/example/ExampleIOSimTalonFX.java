package frc.robot.subsystems.example;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.example.Example.ExampleConstants;
import frc.robot.util.CtreUtil;

public class ExampleIOSimTalonFX extends ExampleIOTalonFX {

    private static final double kSimLoopPeriodSeconds = 0.02;

    private final SingleJointedArmSim intakePivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ExampleConstants.kIntakePivotReduction,
            ExampleConstants.kIntakePivotMOI,
            ExampleConstants.kIntakePivotLengthMeters,
            Math.toRadians(ExampleConstants.kIntakePivotMinDegrees),
            Math.toRadians(ExampleConstants.kIntakePivotMaxDegrees),
            true,
            Math.toRadians(ExampleConstants.kIntakePivotMinDegrees));

    private final FlywheelSim intakeRollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(1),
                    ExampleConstants.kIntakeRollerMOI,
                    ExampleConstants.kIntakeRollerReduction),
            DCMotor.getKrakenX60(1));

    public ExampleIOSimTalonFX() {
        super();
        configureSim();
    }

    private void configureSim() {
        CtreUtil.configureKrakenX60Sim(
                m_intakePivotMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        CtreUtil.configureKrakenX60Sim(
                m_intakeRollerMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
    }

    @Override
    public void updateInputs(ExampleInputs inputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        TalonFXSimState intakePivotState = m_intakePivotMotor.getSimState();
        intakePivotState.setSupplyVoltage(batteryVoltage);

        double intakePivotAppliedVolts = intakePivotState.getMotorVoltageMeasure().baseUnitMagnitude();
        intakePivotSim.setInputVoltage(intakePivotAppliedVolts);
        intakePivotSim.update(kSimLoopPeriodSeconds);

        double intakePivotDegrees = Math.toDegrees(intakePivotSim.getAngleRads());
        double intakePivotDegreesPerSecond =
                Math.toDegrees(intakePivotSim.getVelocityRadPerSec());
        intakePivotState.setRawRotorPosition(intakePivotDegrees / 360.0 * ExampleConstants.kIntakePivotReduction);
        intakePivotState.setRotorVelocity(intakePivotDegreesPerSecond / 360.0 * ExampleConstants.kIntakePivotReduction);

        inputs.intakePivotPositionDegrees = intakePivotDegrees;
        inputs.intakePivotAppliedVolts = intakePivotAppliedVolts;
        inputs.intakePivotStatorCurrentAmps = intakePivotState.getTorqueCurrent();
        inputs.intakePivotSupplyCurrentAmps = intakePivotState.getSupplyCurrent();
        inputs.intakePivotMotorConnected = m_intakePivotMotor.isConnected();

        TalonFXSimState intakeRollerState = m_intakeRollerMotor.getSimState();
        intakeRollerState.setSupplyVoltage(batteryVoltage);

        double intakeRollerAppliedVolts = intakeRollerState.getMotorVoltageMeasure().baseUnitMagnitude();
        intakeRollerSim.setInputVoltage(intakeRollerAppliedVolts);
        intakeRollerSim.update(kSimLoopPeriodSeconds);

        double intakeRollerVelocityRPM = intakeRollerSim.getAngularVelocityRPM();
        intakeRollerState.setRotorVelocity(intakeRollerVelocityRPM / 60.0 * ExampleConstants.kIntakeRollerReduction);

        inputs.intakeRollerVelocityRPM = intakeRollerVelocityRPM;
        inputs.intakeRollerAppliedVolts = intakeRollerAppliedVolts;
        inputs.intakeRollerStatorCurrentAmps = intakeRollerState.getTorqueCurrent();
        inputs.intakeRollerSupplyCurrentAmps = intakeRollerState.getSupplyCurrent();
        inputs.intakeRollerMotorConnected = m_intakeRollerMotor.isConnected();
    }

    @Override
    public void resetEncoder() {
        super.resetEncoder();
        intakePivotSim.setState(Math.toRadians(ExampleConstants.kIntakePivotMinDegrees), 0.0);
    }
}
