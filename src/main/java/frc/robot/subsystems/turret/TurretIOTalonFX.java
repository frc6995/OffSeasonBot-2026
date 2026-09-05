package frc.robot.subsystems.turret;

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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.ConnectionPoll;
import frc.robot.util.CtreUtil;

import static frc.robot.subsystems.turret.Turret.TurretConstants.*;

public class TurretIOTalonFX implements TurretIO {
    //need to specify upper or lower CAN bus
    protected final TalonFX m_turretMotor = new TalonFX(kCANID, Constants.CANBuses.UpperBus); 
    /** Throttles the isConnected() polling below; see ConnectionPoll. */
    private final ConnectionPoll connectionPoll = new ConnectionPoll();

    protected final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
    
    protected StatusSignal<Angle> angleSignal;
    protected StatusSignal<AngularVelocity> velocitySignal;
    protected StatusSignal<Voltage> voltSignal;
    protected StatusSignal<Current> statorCurrentSignal;
    protected StatusSignal<Current> supplyCurrentSignal;

    public TurretIOTalonFX() {
        configMotor();

        angleSignal = m_turretMotor.getPosition();
        velocitySignal = m_turretMotor.getVelocity();
        voltSignal = m_turretMotor.getMotorVoltage();

        statorCurrentSignal = m_turretMotor.getStatorCurrent();
        supplyCurrentSignal = m_turretMotor.getSupplyCurrent();

        // Phoenix publishes current signals far too slowly by default to resolve a brownout; see
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
                .withKA(kA)
                .withKS(kS);
        
        config.SoftwareLimitSwitch = 
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angleToMechanismRotations(kMaxAngle))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(angleToMechanismRotations(kMinAngle));

        config.HardwareLimitSwitch =
            new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(false)
                .withReverseLimitEnable(false);

        CtreUtil.reportIfNotOk("Config Turret", m_turretMotor.getConfigurator().apply(config));
    }

    @Override
    public void resetEncoder() {
        m_turretMotor.setPosition(0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, velocitySignal, voltSignal, statorCurrentSignal, supplyCurrentSignal);

        inputs.angle = mechanismToAngleRotations(angleSignal.getValueAsDouble());
        // SensorToMechanismRatio is configured, so this is mechanism rotations/sec.
        inputs.velocity = mechanismToAngleRotations(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
        // isConnected() is a JNI signal refresh, not a field read, and the Version signal
        // behind it only updates at 4Hz -- polling every loop repeats work. See ConnectionPoll.
        if (connectionPoll.due()) {
            inputs.turretMotorConnected = m_turretMotor.isConnected();
        }
    }

    @Override
    public void setAngle(double angle) {
        double clampedAngle = MathUtil.clamp(angle, kMinAngle, kMaxAngle);

        double rotations = clampedAngle / 360;
        m_turretMotor.setControl(positionRequest.withPosition(rotations));
    }
    
    protected double angleToMotorRotations(double angle) {
        return (angle/360)*kReduction;
    }

    protected double angleToMechanismRotations(double angle) {
        return angle / 360.0;
    }

    protected double mechanismToAngleRotations(double rotations) {
        return rotations * 360.0;
    }

    @Override
    public void disable() {
        m_turretMotor.set(0);
    }
    
}
