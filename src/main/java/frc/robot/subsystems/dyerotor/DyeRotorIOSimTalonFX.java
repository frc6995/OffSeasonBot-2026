package frc.robot.subsystems.dyerotor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import frc.robot.DyeRotorConstants;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.dyerotor.DyeRotor.DyeRotorConstants;

public class DyeRotorIOSimTalonFX extends DyeRotorIOTalonFX {
  private final FlywheelSim spinSim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          DyeRotorConstants.SpinMOI,
          DyeRotorConstants.kSpinReduction),
          
          DCMotor.getKrakenX60(1));
  private final FlywheelSim indexSim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(2),
          DyeRotorConstants.IndexMOI,
          DyeRotorConstants.kIndexReduction),
          DCMotor.getKrakenX60(2));

  public DyeRotorIOSimTalonFX() {
    super();
    configureSim();
  }

  private void configureSim() {
    configureKrakenSim(m_spinMotor.getSimState(), ChassisReference.Clockwise_Positive);
    configureKrakenSim(m_indexerLead.getSimState(), ChassisReference.CounterClockwise_Positive);
    configureKrakenSim(m_indexerFollow.getSimState(), ChassisReference.CounterClockwise_Positive);

  }

  private static void configureKrakenSim(TalonFXSimState simState, ChassisReference orientation) {
    simState.Orientation = orientation;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void updateInputs(DyeRotorInputs inputs) {
    TalonFXSimState spinState = m_spinMotor.getSimState();
    TalonFXSimState indexState = m_indexerLead.getSimState();

    double batteryVoltage = RobotController.getBatteryVoltage();

    spinState.setSupplyVoltage(batteryVoltage);
    indexState.setSupplyVoltage(batteryVoltage);

    double spinAppliedVolts = spinState.getMotorVoltageMeasure().baseUnitMagnitude();
    spinSim.setInputVoltage(spinAppliedVolts);

    double indexAppliedVolts = indexState.getMotorVoltageMeasure().baseUnitMagnitude();
    indexSim.setInputVoltage(indexAppliedVolts);

    spinSim.update(0.02);
    indexSim.update(0.02);

    double spinVelocityRPM = spinSim.getAngularVelocityRPM();

    double indexVelocityRPM = indexSim.getAngularVelocityRPM();
    
    spinState.setRotorVelocity(spinVelocityRPM / 60.0);
    indexState.setRotorVelocity(indexVelocityRPM / 60.0);

    inputs.spinVelocityRPM = spinVelocityRPM;
    inputs.spinAppliedVolts = spinAppliedVolts;
    inputs.spinStatorCurrentAmps = spinState.getTorqueCurrent();
    inputs.spinSupplyCurrentAmps = spinState.getSupplyCurrent();
    inputs.spinMotorConnected = true;

    inputs.indexVelocityRPM = indexVelocityRPM;
    inputs.indexAppliedVolts = indexAppliedVolts;
    inputs.indexStatorCurrentAmps = indexState.getTorqueCurrent();
    inputs.indexSupplyCurrentAmps = indexState.getSupplyCurrent();
    inputs.indexLeadMotorConnected = true;
    inputs.indexFollowerMotorConnected = true;
  }
}