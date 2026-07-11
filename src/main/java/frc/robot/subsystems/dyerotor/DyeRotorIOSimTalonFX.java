package frc.robot.subsystems.dyerotor;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import frc.robot.DyeRotorConstants;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class DyeRotorIOSimTalonFX extends DyeRotorIOTalonFX {
  private final FlywheelSim m_dyeSim =
      new FlywheelSim(
          DCMotor.getKrakenX60(1),
          DyeRotorConstants.kSpinReduction,
          DyeRotorConstants.MOI
          );

  public DyeRotorSim() {
    super();
    configureSim();
  }

  private void configureSim() {
    var simState = m_spindexer.getSimState();
    simState.Orientation = ChassisReference.CounterClockwise_Positive;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void updateInputs(DyeRotorInputs inputs) {
    var simState = m_spindexer.getSimState();

    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVolts = simState.getMotorVoltageMeasure().baseUnitMagnitude();

    m_dyeSim.setInputVoltage(appliedVolts);
    m_dyeSim.update(0.02);

    double spindexerPos = m_dyeSim.getPositionRotations();
    double spindexerVel = m_dyeSim.getVelocityRotationsPerSecond();
    
    simState.setRawRotorPosition(mechanismRotationsToSpinMotorRotations(spindexerPos));
    simState.setRotorVelocity(mechanismRotationsToSpinMotorRotations(spindexerVel));

    inputs.spinPositionRotations = spindexerPos;
    inputs.spinVelocityMetersPerSecond = spindexerVel;
    inputs.spinAppliedVolts = appliedVolts;
    inputs.spinStatorCurrentAmps = Math.abs(m_dyeSim.getCurrentDrawAmps());
    inputs.spinSupplyCurrentAmps = inputs.spinStatorCurrentAmps;
  }
}