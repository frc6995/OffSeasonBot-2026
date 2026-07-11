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

public class DyeRotorIOSimTalonFX extends DyeRotorIOTalonFX {
  private static final double dtSeconds = 0.02;
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

  private double spinMechanismPositionRotations;
  private double indexMechanismPositionRotations;

  public DyeRotorIOSimTalonFX() {
    super();
    configureSim();
  }

  private void configureSim() {
    configureKrakenSim(m_spindexer.getSimState(), ChassisReference.Clockwise_Positive);
    configureKrakenSim(m_indexerLead.getSimState(), ChassisReference.CounterClockwise_Positive);
    configureKrakenSim(m_indexerFollow.getSimState(), ChassisReference.CounterClockwise_Positive);

  }

  private static void configureKrakenSim(TalonFXSimState simState, ChassisReference orientation) {
    simState.Orientation = orientation;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void updateInputs(DyeRotorInputs inputs) {
    TalonFXSimState spinState = m_spindexer.getSimState();
    TalonFXSimState indexLeadState = m_indexerLead.getSimState();
    TalonFXSimState indexFollowerState = m_indexerFollow.getSimState();

    double batteryVoltage = RobotController.getBatteryVoltage();

    spinState.setSupplyVoltage(batteryVoltage);
    indexLeadState.setSupplyVoltage(batteryVoltage);
    indexFollowerState.setSupplyVoltage(batteryVoltage);

    double previousSpinVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
    double previousIndexVelocityRPS = indexSim.getAngularVelocityRPM() / 60.0;

    double spinAppliedVolts = spinState.getMotorVoltageMeasure().baseUnitMagnitude();
    spinSim.setInputVoltage(spinAppliedVolts);

    double indexAppliedVolts = indexLeadState.getMotorVoltageMeasure().baseUnitMagnitude();
    indexSim.setInputVoltage(indexAppliedVolts);

    spinSim.update(dtSeconds);
    indexSim.update(dtSeconds);

    double spinVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;

    double indexVelocityRPS = indexSim.getAngularVelocityRPM() / 60.0;

    spinMechanismPositionRotations += 0.5 * (previousSpinVelocityRPS + spinVelocityRPS) * dtSeconds;
    indexMechanismPositionRotations += 0.5 * (previousIndexVelocityRPS + indexVelocityRPS) * dtSeconds;
    // Deleted: double spinPos = spinSim.getSpinPositionRotations();
    /*
     * 
     * 
     * I need help on updating I honestly don't know and can't find the format:
     */

    updateSpinSim(spinState, spinVelocityRPS);
    updateIndexSim(indexLeadState, indexFollowerState, indexVelocityRPS);

    super.updateInputs((inputs));


    private void updateSpinSim(TalonFXSimState spinState, double mechanismVelocityRPS) {

    }

    private void updateIndexSim(TalonFXSimState indexLeadState, talonFXSimState indexFollowerState, double mechanismVelocityRPS) {

    }
    
    // double spinVelocityRPM = spinSim.getAngularVelocityRPM();
    // double spinVelocityRPS = spinVelocityRPM / 60.0;
    
    // spinState.setRawRotorPosition(mechanismRotationsToSpinMotorRotations(spinPos));
    // spinState.setRotorVelocity(
    //     spinVelocityRPS * DyeRotorConstants.kSpinReduction);    
    // inputs.indexAppliedVolts = indexAppliedVolts;
    // inputs.indexStatorCurrentAmps = Math.abs(indexSim.getCurrentDrawAmps());
    // inputs.indexSupplyCurrentAmps = inputs.indexStatorCurrentAmps;
    
    // inputs.spinPositionRotations = spinPos;
    // inputs.spinVelocityRPM = spinVel;
    // inputs.spinAppliedVolts = spinAppliedVolts;
    // inputs.spinStatorCurrentAmps = Math.abs(spinSim.getCurrentDrawAmps());
    // inputs.spinSupplyCurrentAmps = inputs.spinStatorCurrentAmps;
  }
}