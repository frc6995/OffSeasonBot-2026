package frc.robot.subsystems.Flywheel;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.Flywheel.RealFlywheel;
import frc.robot.subsystems.Flywheel.RealFlywheel.FlywheelConstants;
import static edu.wpi.first.units.Units.RadiansPerSecond;
public class Flywheelsim extends FlywheelIOTalonFX{
    private final FlywheelSim flywheelSim =
      new FlywheelSim (LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1),
          FlywheelConstants.FlywheelMOI,
          FlywheelConstants.kReduction),
          DCMotor.getKrakenX60(1));


     public Flywheelsim() {
    super();
    configureSim();
  }

  private void configureSim() {
   
    configureKrakenSim(m_flywheelLeadMotor.getSimState(), ChassisReference.Clockwise_Positive);
    configureKrakenSim(m_flywheelFollowMotor1.getSimState(), ChassisReference.CounterClockwise_Positive);
    configureKrakenSim(m_flywheelFollowMotor2.getSimState(), ChassisReference.CounterClockwise_Positive);
    configureKrakenSim(m_flywheelFollowMotor3.getSimState(), ChassisReference.CounterClockwise_Positive);
  }

 private static void configureKrakenSim(TalonFXSimState simState, ChassisReference orientation) {
    simState.Orientation = orientation;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

   @Override
  public void updateInputs(FlywheelInputs inputs) {
    var flywheelState = m_flywheelLeadMotor.getSimState();
  
  double batteryVoltage = RobotController.getBatteryVoltage();
  flywheelState.setSupplyVoltage(batteryVoltage);

   double appliedVolts = flywheelState.getMotorVoltageMeasure().baseUnitMagnitude();
    flywheelSim.setInputVoltage(appliedVolts);

    flywheelSim.update(0.02);

  double velocityRPM = flywheelSim.getAngularVelocityRPM();

  flywheelState.setRotorVelocity(velocityRPM/60);


     inputs.velocityRPM = velocityRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = flywheelState.getTorqueCurrent();
    inputs.supplyCurrentAmps = flywheelState.getSupplyCurrent();
    inputs.leadMotorConnected = true;

  }
}


