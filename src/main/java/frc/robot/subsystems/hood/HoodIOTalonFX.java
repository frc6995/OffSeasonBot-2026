package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood.HoodConstants;

public class HoodIOTalonFX implements HoodIO {   

    protected final TalonFX m_hoodMotor = new TalonFX(Hood.HoodConstants.kCANID, Constants.CANBuses.UpperBus); 
    protected final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
    
    protected final StatusSignal<Angle> angleSignal = m_hoodMotor.getPosition();
    protected final StatusSignal<Voltage> voltSignal = m_hoodMotor.getMotorVoltage();
    protected final StatusSignal<Current> statorCurrentSignal = m_hoodMotor.getStatorCurrent();
    protected final StatusSignal<Current> supplyCurrentSignal = m_hoodMotor.getSupplyCurrent();

    public HoodIOTalonFX() {
        configMotor();
    }

    public void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput = 
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        
        config.CurrentLimits = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Hood.HoodConstants.kStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Hood.HoodConstants.kSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true);
        
        config.Feedback = 
            new FeedbackConfigs().withSensorToMechanismRatio(Hood.HoodConstants.kReduction);

        config.Slot0 = 
            new Slot0Configs()
                .withKP(Hood.HoodConstants.kP)
                .withKV(Hood.HoodConstants.kV)
                .withKG(Hood.HoodConstants.kG)
                .withKD(Hood.HoodConstants.kD)
                .withKS(Hood.HoodConstants.kS);
        
        config.SoftwareLimitSwitch = 
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angleToMotorRotations(Hood.HoodConstants.MAX_ANGLE))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(angleToMotorRotations(Hood.HoodConstants.MIN_ANGLE));

        config.HardwareLimitSwitch =
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(false)
                .withReverseLimitEnable(false);

        // need to set these
        config.MotionMagic = 
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(2)
                .withMotionMagicCruiseVelocity(2);

        // TODO replace this with CtreUtil reportIfNotOk
        m_hoodMotor.getConfigurator().apply(config);
    }

    @Override
    public void resetEncoder() {
        m_hoodMotor.setPosition(0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, voltSignal, statorCurrentSignal, supplyCurrentSignal);

        inputs.angle = motorRotationsToAngle(angleSignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
        inputs.hoodMotorConnected = m_hoodMotor.isConnected();
    }

    @Override
    public void setAngle(double angle) {
        double rotations = angle / 360;
        m_hoodMotor.setControl(positionRequest.withPosition(rotations));
    }
    
    /**
     * @param angle
     * 
     * The angle of the hood in degrees
     * @return
     * The number of motor rotations for a given hood angle
     */

    protected double angleToMotorRotations(double angle) {
        return (angle/360.0)*HoodConstants.kReduction;
    }

    protected double motorRotationsToAngle(double rotations) {
        return rotations*(1/HoodConstants.kReduction)*360;
    }

    @Override
    public void disable() {
        this.setAngle(0);
    }

    
}
