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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood.HoodConstants;
import frc.robot.util.ConnectionPoll;
import frc.robot.util.CtreUtil;

public class HoodIOTalonFX implements HoodIO {   

    protected final TalonFX m_hoodMotor = new TalonFX(Hood.HoodConstants.kCANID, Constants.CANBuses.UpperBus); 
    /** Throttles the isConnected() polling below; see ConnectionPoll. */
    private final ConnectionPoll connectionPoll = new ConnectionPoll();

    protected final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
    
    protected final StatusSignal<Angle> angleSignal = m_hoodMotor.getPosition();
    protected final StatusSignal<Voltage> voltSignal = m_hoodMotor.getMotorVoltage();
    protected final StatusSignal<Current> statorCurrentSignal = m_hoodMotor.getStatorCurrent();
    protected final StatusSignal<Current> supplyCurrentSignal = m_hoodMotor.getSupplyCurrent();

    public HoodIOTalonFX() {
        configMotor();
        // Current signals are published at an explicit rate rather than Phoenix's default,
        // which is not guaranteed fast enough to resolve a brownout. See
        // CtreUtil.kCurrentSignalFrequencyHz.
        CtreUtil.setCurrentSignalFrequency(statorCurrentSignal, supplyCurrentSignal);
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
                .withForwardSoftLimitThreshold(angleToMechanismRotations(Hood.HoodConstants.MAX_ANGLE))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(angleToMechanismRotations(Hood.HoodConstants.MIN_ANGLE));

        config.HardwareLimitSwitch =
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(false)
                .withReverseLimitEnable(false);

        CtreUtil.reportIfNotOk("Config hood", m_hoodMotor.getConfigurator().apply(config));
    }

    @Override
    public void resetEncoder() {
        m_hoodMotor.setPosition(0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, voltSignal, statorCurrentSignal, supplyCurrentSignal);

        inputs.angle = mechanismRotationsToAngle(angleSignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
        // isConnected() is a JNI signal refresh, not a field read, and the Version signal
        // behind it only updates at 4Hz -- polling every loop repeats work. See ConnectionPoll.
        if (connectionPoll.due()) {
            inputs.hoodMotorConnected = m_hoodMotor.isConnected();
        }
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

    protected double angleToMechanismRotations(double angle) {
        return angle / 360.0;
    }

    protected double mechanismRotationsToAngle(double rotations) {
        return rotations * 360.0;
    }

    @Override
    public void disable() {
        this.setAngle(0);
    }

    
}
