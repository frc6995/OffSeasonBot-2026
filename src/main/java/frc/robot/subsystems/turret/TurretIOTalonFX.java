package frc.robot.subsystems.turret;

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
import static frc.robot.subsystems.turret.Turret.TurretConstants.*;

public class TurretIOTalonFX implements TurretIO{
    //need to specify upper or lower CAN bus
    protected final TalonFX m_turretMotor = new TalonFX(kCANID); 
    protected final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    
    protected StatusSignal<Angle> angleSignal;
    protected StatusSignal<Voltage> voltSignal;
    protected StatusSignal<Current> statorCurrentSignal;
    protected StatusSignal<Current> supplyCurrentSignal;

    public TurretIOTalonFX() {
        configMotor();

        angleSignal = m_turretMotor.getPosition();
        voltSignal = m_turretMotor.getMotorVoltage();

        statorCurrentSignal = m_turretMotor.getStatorCurrent();
        supplyCurrentSignal = m_turretMotor.getSupplyCurrent();
    }

    public void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput = 
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        
        config.CurrentLimits = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(kSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true);
        
        config.Feedback = 
            new FeedbackConfigs().withSensorToMechanismRatio(kReduction);

        config.Slot0 = 
            new Slot0Configs()
                .withKP(kP)
                .withKV(kV)
                .withKA(kA);
        
        config.SoftwareLimitSwitch = 
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angleToRotations(0)) //fix
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0);

        config.HardwareLimitSwitch =
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(true)
                .withReverseLimitEnable(true);

        //TODO replace this with CtreUtil reportIfNotOk
        m_turretMotor.getConfigurator().apply(config);
    }

    @Override
    public void resetEncoder() {
        m_turretMotor.setPosition(0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, voltSignal, statorCurrentSignal, supplyCurrentSignal);

        // inputs.angle = rotationsToAngle(angleSignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    }

    @Override
    public void setAngle(double angle) {

    }
    
    //fix
    protected double angleToRotations(double angle) {
        return 0.0;
    }

    //fix
    protected double rotationsToAngle(double rotations) {
        return 0.0;
    }

    @Override
    public void stop() {
        m_turretMotor.stopMotor();
    }
    
}
