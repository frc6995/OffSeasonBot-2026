package frc.robot.subsystems.turret;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSimTalonFX extends TurretIOTalonFX {

  private final SingleJointedArmSim m_TurretSim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1), 
        Turret.TurretConstants.kReduction, 
        Turret.TurretConstants.kMOI, //Need MOI
        Turret.TurretConstants.kLength, 
        Math.toRadians(Turret.TurretConstants.kMinAngle), 
        Math.toRadians(Turret.TurretConstants.kMaxAngle), 
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

    m_TurretSim.setInputVoltage(appliedVolts);

    double turretPosition = Math.toDegrees(m_TurretSim.getAngleRads());

    simState.setRawRotorPosition(angleToRotations(turretPosition));

    inputs.angle = turretPosition;
    inputs.appliedVolts =  appliedVolts;
    inputs.supplyCurrent = simState.getSupplyCurrent();
    inputs.statorCurrent = simState.getTorqueCurrent();

    m_TurretSim.update(0.02);

  }
}