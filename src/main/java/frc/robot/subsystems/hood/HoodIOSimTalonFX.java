package frc.robot.subsystems.hood;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.CtreUtil;

public class HoodIOSimTalonFX extends HoodIOTalonFX {
    
    private final SingleJointedArmSim m_HoodSim = 
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1), 
            Hood.HoodConstants.kReduction, 
            Hood.HoodConstants.kMOI, // kg m^2 
            Hood.HoodConstants.kHoodLength,// m
            Math.toRadians(Hood.HoodConstants.MIN_ANGLE), 
            Math.toRadians(Hood.HoodConstants.MAX_ANGLE), 
            true, 
            0);

    public HoodIOSimTalonFX() {
        super();
        configureSim();
    }

    private void configureSim() {
        CtreUtil.configureKrakenX44Sim(
                m_hoodMotor.getSimState(), ChassisReference.CounterClockwise_Positive);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        m_HoodSim.update(0.02);

        var simState = m_hoodMotor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double appliedVolts = simState.getMotorVoltageMeasure().baseUnitMagnitude();

        m_HoodSim.setInputVoltage(appliedVolts);

        double hoodPosition = Math.toDegrees(m_HoodSim.getAngleRads());

        simState.setRawRotorPosition(angleToMotorRotations(hoodPosition));

        inputs.angle = hoodPosition;
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrent = simState.getTorqueCurrent();
        inputs.supplyCurrent = simState.getSupplyCurrent();
    }
}
