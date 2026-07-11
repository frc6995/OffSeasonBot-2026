package frc.robot.subsystems.turret;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSimTalonFX extends TurretIOTalonFX{
    private final SingleJointedArmSim m_turretsim = new SingleJointedArmSim(
    DCMotor.getKrakenX44(1), 
    Turret.TurretConstants.kReduction, 
    Turret.TurretConstants.kMOI, //Need MOI
    Turret.TurretConstants.kLength, 
    Math.toRadians(0), 
    Math.toRadians(720), 
    false, 
    0, 
    null);



  public TurretIOSimTalonFX() {
    super();
    configureSim();
  }

  private void configureSim() {
    var simState = m_turretMotor.getSimState();
    simState.Orientation = ChassisReference.CounterClockwise_Positive;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var simState = m_turretMotor.getSimState();

    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVolts = simState.getMotorVoltageMeasure().baseUnitMagnitude();

    m_turretsim.setInputVoltage(appliedVolts);
    inputs.appliedVolts =  appliedVolts;
    inputs.statorCurrent = Math.abs(m_turretsim.getCurrentDrawAmps());
    inputs.supplyCurrent = inputs.supplyCurrent;
    m_turretsim.update(0.02);

  }
}