package frc.robot.subsystems.arm;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.Arm.ArmConstants;
import frc.robot.util.CtreUtil;

public class ArmIOSimTalonFX extends ArmIOTalonFX {

    private static final double kSimLoopPeriodSeconds = 0.02;

    private final ElevatorSim elevatorSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                    DCMotor.getKrakenX44(2),
                    ArmConstants.kElevatorCarriageMassKg,
                    ArmConstants.kElevatorDrumRadiusMeters,
                    ArmConstants.kElevatorReduction),
            DCMotor.getKrakenX44(2),
            ArmConstants.kElevatorMinMeters,
            ArmConstants.kElevatorMaxMeters,
            true,
            ArmConstants.kElevatorMinMeters);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            ArmConstants.kArmReduction,
            ArmConstants.kArmMOI,
            ArmConstants.kArmLengthMeters,
            Math.toRadians(ArmConstants.kArmMinDegrees),
            Math.toRadians(ArmConstants.kArmMaxDegrees),
            true,
            Math.toRadians(ArmConstants.kArmMinDegrees));

    private final FlywheelSim handSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(1),
                    ArmConstants.kHandMOI,
                    ArmConstants.kHandReduction),
            DCMotor.getKrakenX60(1));

    public ArmIOSimTalonFX() {
        super();
        configureSim();
    }

    private void configureSim() {
        CtreUtil.configureKrakenX44Sim(
                m_elevatorLeadMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        CtreUtil.configureKrakenX44Sim(
                m_elevatorFollowerMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        CtreUtil.configureKrakenX44Sim(
                m_armMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        CtreUtil.configureKrakenX60Sim(
                m_handMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        TalonFXSimState elevatorState = m_elevatorLeadMotor.getSimState();
        elevatorState.setSupplyVoltage(batteryVoltage);
        m_elevatorFollowerMotor.getSimState().setSupplyVoltage(batteryVoltage);

        double elevatorAppliedVolts = elevatorState.getMotorVoltageMeasure().baseUnitMagnitude();
        elevatorSim.setInputVoltage(elevatorAppliedVolts);
        elevatorSim.update(kSimLoopPeriodSeconds);

        double elevatorMeters = elevatorSim.getPositionMeters();
        double elevatorMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
        elevatorState.setRawRotorPosition(
                elevatorMeters
                        / ArmConstants.kElevatorDrumCircumferenceMeters
                        * ArmConstants.kElevatorReduction);
        elevatorState.setRotorVelocity(
                elevatorMetersPerSecond
                        / ArmConstants.kElevatorDrumCircumferenceMeters
                        * ArmConstants.kElevatorReduction);

        inputs.elevatorPositionMeters = elevatorMeters;
        inputs.elevatorAppliedVolts = elevatorAppliedVolts;
        inputs.elevatorStatorCurrentAmps = elevatorState.getTorqueCurrent();
        inputs.elevatorSupplyCurrentAmps = elevatorState.getSupplyCurrent();
        inputs.elevatorLeadMotorConnected = m_elevatorLeadMotor.isConnected();
        inputs.elevatorFollowerMotorConnected = m_elevatorFollowerMotor.isConnected();

        TalonFXSimState armState = m_armMotor.getSimState();
        armState.setSupplyVoltage(batteryVoltage);

        double armAppliedVolts = armState.getMotorVoltageMeasure().baseUnitMagnitude();
        armSim.setInputVoltage(armAppliedVolts);
        armSim.update(kSimLoopPeriodSeconds);

        double armDegrees = Math.toDegrees(armSim.getAngleRads());
        double armDegreesPerSecond =
                Math.toDegrees(armSim.getVelocityRadPerSec());
        armState.setRawRotorPosition(armDegrees / 360.0 * ArmConstants.kArmReduction);
        armState.setRotorVelocity(armDegreesPerSecond / 360.0 * ArmConstants.kArmReduction);

        inputs.armPositionDegrees = armDegrees;
        inputs.armAppliedVolts = armAppliedVolts;
        inputs.armStatorCurrentAmps = armState.getTorqueCurrent();
        inputs.armSupplyCurrentAmps = armState.getSupplyCurrent();
        inputs.armMotorConnected = m_armMotor.isConnected();

        TalonFXSimState handState = m_handMotor.getSimState();
        handState.setSupplyVoltage(batteryVoltage);

        double handAppliedVolts = handState.getMotorVoltageMeasure().baseUnitMagnitude();
        handSim.setInputVoltage(handAppliedVolts);
        handSim.update(kSimLoopPeriodSeconds);

        double handVelocityRPM = handSim.getAngularVelocityRPM();
        handState.setRotorVelocity(handVelocityRPM / 60.0 * ArmConstants.kHandReduction);

        inputs.handVelocityRPM = handVelocityRPM;
        inputs.handAppliedVolts = handAppliedVolts;
        inputs.handStatorCurrentAmps = handState.getTorqueCurrent();
        inputs.handSupplyCurrentAmps = handState.getSupplyCurrent();
        inputs.handMotorConnected = m_handMotor.isConnected();
    }

    @Override
    public void resetEncoder() {
        super.resetEncoder();
        elevatorSim.setState(ArmConstants.kElevatorMinMeters, 0.0);
        armSim.setState(Math.toRadians(ArmConstants.kArmMinDegrees), 0.0);
    }
}
