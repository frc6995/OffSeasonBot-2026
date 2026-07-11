package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOTalonFX implements HoodIO{   
    //need to specify upper or lower CAN bus
    protected final TalonFX m_hoodMotor = new TalonFX(Hood.HoodConstants.kCANID); 
    protected final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    
    protected StatusSignal<Angle> angleSignal;
    protected StatusSignal<Voltage> voltSignal;
    protected StatusSignal<Current> statorCurrentSignal;
    protected StatusSignal<Current> supplyCurrentSignal;

    public HoodIOTalonFX() {
        configMotor();

        angleSignal = m_hoodMotor.getPosition();
        voltSignal = m_hoodMotor.getMotorVoltage();

        statorCurrentSignal = m_hoodMotor.getStatorCurrent();
        supplyCurrentSignal = m_hoodMotor.getSupplyCurrent();
    }

    public void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput = 
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
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
                .withKA(Hood.HoodConstants.kA);
        
        config.SoftwareLimitSwitch = 
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angleToRotations(42.5))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0);

        config.HardwareLimitSwitch =
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(true)
                .withReverseLimitEnable(true);

        //TODO replace this with CtreUtil reportIfNotOk
        m_hoodMotor.getConfigurator().apply(config);
    }

    @Override
    public void resetEncoder() {
        m_hoodMotor.setPosition(0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, voltSignal, statorCurrentSignal, supplyCurrentSignal);

        inputs.angle = rotationsToAngle(angleSignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    }

    @Override
    public void setAngle(double angle) {
        m_hoodMotor.setControl(positionRequest.withPosition(angleToRotations(angle)));
    }
    
    /**
     * @param angle
     * 
     * The angle of the hood in degrees
     * @return
     * The number of motor rotations for a given hood angle
     */

    protected double angleToRotations(double angle) {
        return (angle/360)*(Hood.HoodConstants.kReduction);
    }

    protected double rotationsToAngle(double rotations) {
        return rotations*(1/Hood.HoodConstants.kReduction)*360;
    }

    @Override
    public void disable() {
        this.setAngle(0);
    }

    
}
