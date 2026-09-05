package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.CtreUtil;

public class FlywheelIOSimTalonFX extends FlywheelIOTalonFX {
  private final FlywheelSim flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
      DCMotor.getKrakenX44(4),
      FlywheelConstants.FlywheelMOI,
      FlywheelConstants.kReduction),
      DCMotor.getKrakenX44(4));

  public FlywheelIOSimTalonFX() {
    super();
    configureSim();
  }

  private void configureSim() {

    CtreUtil.configureKrakenX44Sim(m_flywheelLeadMotor.getSimState(), ChassisReference.Clockwise_Positive);
    // CtreUtil.configureKrakenX44Sim(m_flywheelFollowMotor1.getSimState(),ChassisReference.Clockwise_Positive);
    // CtreUtil.configureKrakenX44Sim(m_flywheelFollowMotor2.getSimState(),ChassisReference.CounterClockwise_Positive);
    // CtreUtil.configureKrakenX44Sim(m_flywheelFollowMotor3.getSimState(),ChassisReference.CounterClockwise_Positive);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    var flywheelLeadState = m_flywheelLeadMotor.getSimState();
    // var flywheelFollowerState1 = m_flywheelFollowMotor1.getSimState();
    // var flywheelFollowerState2 = m_flywheelFollowMotor2.getSimState();
    // var flywheelFollowerState3 = m_flywheelFollowMotor3.getSimState();

    double batteryVoltage = RobotController.getBatteryVoltage();
    flywheelLeadState.setSupplyVoltage(batteryVoltage);
    // flywheelFollowerState1.setSupplyVoltage(batteryVoltage);
    // flywheelFollowerState2.setSupplyVoltage(batteryVoltage);
    // flywheelFollowerState3.setSupplyVoltage(batteryVoltage);
    double appliedVolts = flywheelLeadState.getMotorVoltageMeasure().baseUnitMagnitude();
    flywheelSim.setInputVoltage(appliedVolts);

    flywheelSim.update(0.02);

    double velocityRPM = flywheelSim.getAngularVelocityRPM();

    flywheelLeadState.setRotorVelocity(velocityRPM / 60);
    // flywheelFollowerState1.setRotorVelocity(velocityRPM/60);
    // flywheelFollowerState2.setRotorVelocity(velocityRPM/60);
    // flywheelFollowerState3.setRotorVelocity(velocityRPM/60);

    inputs.velocityRPM = velocityRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = flywheelLeadState.getTorqueCurrent();
    inputs.supplyCurrentAmps = flywheelLeadState.getSupplyCurrent();

    // Only the lead motor's sim state is modelled (the follower sim states are commented out
    // above), so each follower is reported as drawing what the lead draws. That is an
    // approximation, not a measurement: on the real robot the followers share a setpoint but not a
    // load. It exists so simulation produces non-zero, correctly-shaped power data to develop
    // tools/power_analysis against - do not read sim flywheel current as a real number.
    for (int i = 0; i < FlywheelIO.kMotorCount; i++) {
      inputs.motorStatorCurrentAmps[i] = inputs.statorCurrentAmps;
      inputs.motorSupplyCurrentAmps[i] = inputs.supplyCurrentAmps;
    }

    inputs.leadMotorConnected = m_flywheelLeadMotor.isConnected();
    inputs.followerMotor1Connected = m_flywheelFollowMotor1.isConnected();
    inputs.followerMotor2Connected = m_flywheelFollowMotor2.isConnected();
    inputs.followerMotor3Connected = m_flywheelFollowMotor3.isConnected();
  }
}