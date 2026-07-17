package frc.robot.subsystems.hood;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSimTalonFX extends HoodIOTalonFX{
    
    private final SingleJointedArmSim m_HoodSim = 
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1), 
            Hood.HoodConstants.kReduction, 
            Hood.HoodConstants.kMOI, //in^2 lbs 
            Hood.HoodConstants.kHoodLength,//in 
            Math.toRadians(Hood.HoodConstants.MIN_ANGLE), 
            Math.toRadians(Hood.HoodConstants.MAX_ANGLE), 
            true, 
            0, 
            null);

    public HoodIOSimTalonFX(){
        super();
        configureSim();
    }

    
    private void configureSim() {
        var simState = m_hoodMotor.getSimState();
        simState.Orientation = ChassisReference.CounterClockwise_Positive;
        simState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
    

    @Override
    public void updateInputs(HoodIOInputs inputs){
        var simState = m_hoodMotor.getSimState();
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double appliedVolts = simState.getMotorVoltageMeasure().baseUnitMagnitude();

        m_HoodSim.setInputVoltage(appliedVolts);

        double hoodPosition = Math.toDegrees(m_HoodSim.getAngleRads());

        simState.setRawRotorPosition(angleToRotations(hoodPosition));

        inputs.angle = hoodPosition;
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrent = simState.getSupplyCurrent();
        inputs.supplyCurrent = simState.getTorqueCurrent();

        m_HoodSim.update(0.02);

    }
}
