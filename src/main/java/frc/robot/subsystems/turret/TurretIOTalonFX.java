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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.util.CtreUtil;
import frc.robot.util.TurretFeedforward;

import static frc.robot.subsystems.turret.Turret.TurretConstants.*;

public class TurretIOTalonFX implements TurretIO {
    //need to specify upper or lower CAN bus
    protected final TalonFX m_turretMotor = new TalonFX(kCANID, Constants.CANBuses.UpperBus);

    private final TurretFeedforward m_feedforward;

    protected final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    protected StatusSignal<Angle> angleSignal;
    protected StatusSignal<AngularVelocity> velocitySignal;
    protected StatusSignal<Voltage> voltSignal;
    protected StatusSignal<Current> statorCurrentSignal;
    protected StatusSignal<Current> supplyCurrentSignal;

    protected double cachedAngle = 0;

    public TurretIOTalonFX() {
        configMotor();

        angleSignal = m_turretMotor.getPosition();
        velocitySignal = m_turretMotor.getVelocity();
        voltSignal = m_turretMotor.getMotorVoltage();

        statorCurrentSignal = m_turretMotor.getStatorCurrent();
        supplyCurrentSignal = m_turretMotor.getSupplyCurrent();

        m_feedforward = new TurretFeedforward(
            TurretConstants.kSpringForceN,
            TurretConstants.kEChainBaseWidth / 2.0,
            TurretConstants.kEChainBaseLength / 2.0,
            TurretConstants.kNMPerVolt,
            -135.612
        );

        // Current signals are published at an explicit rate rather than Phoenix's default,
        // which is not guaranteed fast enough to resolve a brownout. See
        // CtreUtil.kCurrentSignalFrequencyHz.
        CtreUtil.setCurrentSignalFrequency(statorCurrentSignal, supplyCurrentSignal);

        // Must come before the optimize below, and must cover every signal updateInputs()
        // refreshes - anything left out silently drops to 4 Hz. For this motor that would mean
        // selectClosestAngle() and the vision camera transform running on a stale angle.
        BaseStatusSignal.setUpdateFrequencyForAll(
                CtreUtil.kMechanismSignalFrequencyHz, angleSignal, velocitySignal, voltSignal);

        // Everything else this motor publishes - duty cycle, torque current, temperatures,
        // closed-loop telemetry, fault frames - is never read here. On CAN FD all of it defaults
        // to 100 Hz, so Phoenix's receive thread decodes it every loop for nothing.
        CtreUtil.reportIfNotOk("Turret optimize bus utilization",
                ParentDevice.optimizeBusUtilizationForAll(m_turretMotor));
    }

    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput = 
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        
        config.CurrentLimits = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(kSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true);
        
        config.Feedback = 
            new FeedbackConfigs()
                // Phoenix 6 only writes config fields that are explicitly set, so a stale
                // FusedCANcoder/RemoteCANcoder source saved in the Talon's flash (e.g. from
                // Phoenix Tuner) survives this apply() and makes the turret boot at the
                // absolute reading (~0.53 rot) instead of 0. Forcing RotorSensor overrides
                // it on every boot so position starts at 0.
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(kReduction);

        config.Slot0 =
            new Slot0Configs()
                .withKP(kP)
                .withKV(kV)
                .withKA(kA)
                .withKS(kS);

        config.MotionMagic =
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(angleToMechanismRotations(kCruiseVelocityDegPerSec))
                .withMotionMagicAcceleration(angleToMechanismRotations(kMaxAccelerationDegPerSec2));

        config.SoftwareLimitSwitch =
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angleToMechanismRotations(kMaxAngleDeg))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(angleToMechanismRotations(kMinAngleDeg));

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

        cachedAngle = angleSignal.getValueAsDouble();

        inputs.angle = mechanismToAngleDegrees(cachedAngle);
        // SensorToMechanismRatio is configured, so this is mechanism rotations/sec.
        inputs.velocity = mechanismToAngleDegrees(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = voltSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    }

    @Override
    public void setAngle(double angle) {
        double clampedAngle = MathUtil.clamp(angle, kMinAngleDeg, kMaxAngleDeg);

        double rotations = clampedAngle / 360;
        // positionRequest.FeedForward = -m_feedforward.calculate(cachedAngle * Math.PI * 2.0);
        m_turretMotor.setControl(positionRequest.withPosition(rotations));
    }
    
    protected double angleToMotorRotations(double angle) {
        return (angle/360)*kReduction;
    }

    protected double angleToMechanismRotations(double angle) {
        return angle / 360.0;
    }

    protected double mechanismToAngleDegrees(double rotations) {
        return rotations * 360.0;
    }

    @Override
    public void disable() {
        m_turretMotor.set(0);
    }
}
