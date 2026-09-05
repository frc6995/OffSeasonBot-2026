package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.intake.Intake.IntakeConstants;
import frc.robot.util.CtreUtil;

public class IntakeIOSimTalonFX extends IntakeIOTalonFX {
    private static final double kSimLoopPeriodSeconds = 0.02;
     public static final double kExtensionMOI = 0.07;
    private static final double kExtensionCarriageMassKg = 2.0;
    // Derived from the real IO's directly-measured drum circumference (IntakeConstants
    // .kDrumCircumferenceMeters) rather than a separately-estimated radius, so the ElevatorSim's
    // physics and the sensor conversion below always agree with real hardware by construction -
    // no more maintaining two numbers for the same physical drum.
    private static final double kExtensionDrumRadiusMeters =
            IntakeConstants.kDrumCircumferenceMeters / (2.0 * Math.PI);

    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(2),
                    IntakeConstants.kRollerMOI,
                    IntakeConstants.kRollerReduction),
            DCMotor.getKrakenX60(2));

    private final FlywheelSim kickerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(1),
                    IntakeConstants.kKickerMOI,
                    IntakeConstants.kKickerReduction),
            DCMotor.getKrakenX60(1));

    private final ElevatorSim extensionSim = new ElevatorSim(
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

    public IntakeIOSimTalonFX() {
        super();
        configureSim();
    }

    private void configureSim() {
        CtreUtil.configureKrakenX60Sim(m_rollerLeadMotor.getSimState(), ChassisReference.Clockwise_Positive);
        CtreUtil.configureKrakenX60Sim(m_rollerFollowerMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        CtreUtil.configureKrakenX60Sim(m_extensionLeadMotor.getSimState(), ChassisReference.Clockwise_Positive);
        CtreUtil.configureKrakenX60Sim(m_extensionFollowerMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
        CtreUtil.configureKrakenX60Sim(m_kickerMotor.getSimState(), ChassisReference.Clockwise_Positive);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        TalonFXSimState rollerState = m_rollerLeadMotor.getSimState();
        TalonFXSimState followerRollerState = m_rollerFollowerMotor.getSimState();
        TalonFXSimState extensionState = m_extensionLeadMotor.getSimState();
        TalonFXSimState followerExtensionState = m_extensionFollowerMotor.getSimState();
        TalonFXSimState kickerState = m_kickerMotor.getSimState();

        double batteryVoltage = RobotController.getBatteryVoltage();

        rollerState.setSupplyVoltage(batteryVoltage);
        followerRollerState.setSupplyVoltage(batteryVoltage);
        extensionState.setSupplyVoltage(batteryVoltage);
        followerExtensionState.setSupplyVoltage(batteryVoltage);
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

        rollerState.setRotorVelocity(rollerRotationsToMotorRotations(rollerVelocityRPM / 60.0));
        kickerState.setRotorVelocity(kickerRotationsToMotorRotations(kickerVelocityRPM / 60.0));

        extensionState.setRawRotorPosition(metersToMotorRotations(extensionPositionMeters));
        extensionState.setRotorVelocity(metersToMotorRotations(extensionVelocityMetersPerSecond));

        inputs.rollerVelocityRPM = rollerVelocityRPM;
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerStatorCurrentAmps = rollerState.getTorqueCurrent();
        inputs.rollerSupplyCurrentAmps = rollerState.getSupplyCurrent();
        // Only the lead motor's sim state is modelled, so the follower is reported as drawing what
        // the lead draws. An approximation to give simulation correctly-shaped power data for
        // developing tools/power_analysis, not a measurement - see FlywheelIOSimTalonFX.
        inputs.rollerMotorStatorCurrentAmps[0] = inputs.rollerStatorCurrentAmps;
        inputs.rollerMotorStatorCurrentAmps[1] = inputs.rollerStatorCurrentAmps;
        inputs.rollerMotorSupplyCurrentAmps[0] = inputs.rollerSupplyCurrentAmps;
        inputs.rollerMotorSupplyCurrentAmps[1] = inputs.rollerSupplyCurrentAmps;
        inputs.rollerLeadMotorConnected = m_rollerLeadMotor.isConnected();
        inputs.rollerFollowerMotorConnected = m_rollerFollowerMotor.isConnected();

        inputs.extensionPositionMeters = extensionPositionMeters;
        inputs.extensionAppliedVolts = extensionAppliedVolts;
        inputs.extensionStatorCurrentAmps = extensionState.getTorqueCurrent();
        inputs.extensionSupplyCurrentAmps = extensionState.getSupplyCurrent();
        inputs.extensionMotorStatorCurrentAmps[0] = inputs.extensionStatorCurrentAmps;
        inputs.extensionMotorStatorCurrentAmps[1] = inputs.extensionStatorCurrentAmps;
        inputs.extensionMotorSupplyCurrentAmps[0] = inputs.extensionSupplyCurrentAmps;
        inputs.extensionMotorSupplyCurrentAmps[1] = inputs.extensionSupplyCurrentAmps;
        inputs.extensionLeadMotorConnected = m_extensionLeadMotor.isConnected();
        inputs.extensionFollowerMotorConnected = m_extensionFollowerMotor.isConnected();

        inputs.kickerVelocityRPM = kickerVelocityRPM;
        inputs.kickerAppliedVolts = kickerAppliedVolts;
        inputs.kickerStatorCurrentAmps = kickerState.getTorqueCurrent();
        inputs.kickerSupplyCurrentAmps = kickerState.getSupplyCurrent();
        inputs.kickerMotorConnected = m_kickerMotor.isConnected();
    }

    @Override
    public void resetEncoder() {
        super.resetEncoder();
        extensionSim.setState(IntakeConstants.kExtensionMinMeters, 0.0);
    }

    protected static double kickerRotationsToMotorRotations(double rotations) {
        return rotations * IntakeConstants.kKickerReduction;
    }

    protected static double rollerRotationsToMotorRotations(double rotations) {
        return rotations * IntakeConstants.kRollerReduction;
    }
}