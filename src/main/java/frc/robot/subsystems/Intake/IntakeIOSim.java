package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Intake.Intake.IntakeConstants;

public class IntakeIOSim extends IntakeIOTalonFX {
    private static final double kSimLoopPeriodSeconds = 0.02;
    private static final double kRollerMOI = 0.001;
    private static final double kKickerMOI = 0.001;
    private static final double kExtensionCarriageMassKg = 2.0;
    private static final double kExtensionDrumRadiusMeters = 0.019;

    private final FlywheelSim rollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                kRollerMOI,
                IntakeConstants.kRollerReduction),
            DCMotor.getKrakenX60(2));

    private final FlywheelSim kickerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                kKickerMOI,
                IntakeConstants.kKickerReduction),
            DCMotor.getKrakenX60(1));

    private final ElevatorSim extensionSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                kExtensionCarriageMassKg,
                kExtensionDrumRadiusMeters,
                IntakeConstants.kExtensionReduction),
            DCMotor.getKrakenX60(2),
            IntakeConstants.kExtensionMinMeters,
            IntakeConstants.kExtensionMaxMeters,
            false,
            IntakeConstants.kExtensionMinMeters);

    public IntakeIOSim() {
        super();
        configureSim();
    }

    private void configureSim() {
        configureKrakenSim(m_rollerLeadMotor.getSimState(), ChassisReference.Clockwise_Positive);
        configureKrakenSim(m_rollerFollowerMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        configureKrakenSim(m_extensionLeadMotor.getSimState(), ChassisReference.Clockwise_Positive);
        configureKrakenSim(m_extensionFollowerMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        configureKrakenSim(m_kickerMotor.getSimState(), ChassisReference.Clockwise_Positive);
    }

    private static void configureKrakenSim(TalonFXSimState simState, ChassisReference orientation) {
        simState.Orientation = orientation;
        simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        TalonFXSimState rollerState = m_rollerLeadMotor.getSimState();
        TalonFXSimState rollerFollowerState = m_rollerFollowerMotor.getSimState();
        TalonFXSimState extensionState = m_extensionLeadMotor.getSimState();
        TalonFXSimState extensionFollowerState = m_extensionFollowerMotor.getSimState();
        TalonFXSimState kickerState = m_kickerMotor.getSimState();

        double batteryVoltage = RobotController.getBatteryVoltage();
        rollerState.setSupplyVoltage(batteryVoltage);
        rollerFollowerState.setSupplyVoltage(batteryVoltage);
        extensionState.setSupplyVoltage(batteryVoltage);
        extensionFollowerState.setSupplyVoltage(batteryVoltage);
        kickerState.setSupplyVoltage(batteryVoltage);

        double rollerAppliedVolts = rollerState.getMotorVoltageMeasure().baseUnitMagnitude();
        double extensionAppliedVolts = extensionState.getMotorVoltageMeasure().baseUnitMagnitude();
        double kickerAppliedVolts = kickerState.getMotorVoltageMeasure().baseUnitMagnitude();

        rollerSim.setInputVoltage(rollerAppliedVolts);
        extensionSim.setInputVoltage(extensionAppliedVolts);
        kickerSim.setInputVoltage(kickerAppliedVolts);

        rollerSim.update(kSimLoopPeriodSeconds);
        extensionSim.update(kSimLoopPeriodSeconds);
        kickerSim.update(kSimLoopPeriodSeconds);

        double rollerVelocityRPM = rollerSim.getAngularVelocityRPM();
        double kickerVelocityRPM = kickerSim.getAngularVelocityRPM();
        double extensionPositionMeters = extensionSim.getPositionMeters();
        double extensionVelocityMetersPerSecond = extensionSim.getVelocityMetersPerSecond();

        rollerState.setRotorVelocity(rollerVelocityRPM / 60.0);
        rollerFollowerState.setRotorVelocity(-rollerVelocityRPM / 60.0);
        kickerState.setRotorVelocity(kickerVelocityRPM / 60.0);

        extensionState.setRawRotorPosition(metersToMotorRotations(extensionPositionMeters));
        extensionState.setRotorVelocity(metersToMotorRotations(extensionVelocityMetersPerSecond));
        extensionFollowerState.setRawRotorPosition(-metersToMotorRotations(extensionPositionMeters));
        extensionFollowerState.setRotorVelocity(-metersToMotorRotations(extensionVelocityMetersPerSecond));

        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerStatorCurrentAmps = rollerState.getTorqueCurrent();
        inputs.rollerSupplyCurrentAmps = rollerState.getSupplyCurrent();
        inputs.rollerLeadMotorConnected = true;
        inputs.rollerFollowerMotorConnected = true;

        inputs.extensionAppliedVolts = extensionAppliedVolts;
        inputs.extensionStatorCurrentAmps = extensionState.getTorqueCurrent();
        inputs.extensionSupplyCurrentAmps = extensionState.getSupplyCurrent();
        inputs.extensionLeadMotorConnected = true;
        inputs.extensionFollowerMotorConnected = true;

        inputs.kickerAppliedVolts = kickerAppliedVolts;
        inputs.kickerStatorCurrentAmps = kickerState.getTorqueCurrent();
        inputs.kickerSupplyCurrentAmps = kickerState.getSupplyCurrent();
        inputs.kickerMotorConnected = true;
    }
}
