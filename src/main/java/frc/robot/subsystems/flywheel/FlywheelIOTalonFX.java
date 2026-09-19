package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelConstants;
import frc.robot.Constants.CANBuses;
import frc.robot.util.ArrayUtil;
import frc.robot.util.CtreUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

  public FlywheelIOTalonFX() {
    configureMotors();
    // Current signals are published at an explicit rate rather than Phoenix's default,
    // which is not guaranteed fast enough to resolve a brownout. See
    // CtreUtil.kCurrentSignalFrequencyHz.
    CtreUtil.setCurrentSignalFrequency(
        ArrayUtil.concat(m_supplyCurrentSignals, m_statorCurrentSignals));

    // Must come before the optimize below, and must cover every signal updateInputs() refreshes -
    // anything left out silently drops to 4 Hz. Only the lead motor's velocity and voltage are
    // read; the followers contribute nothing beyond the currents set above.
    BaseStatusSignal.setUpdateFrequencyForAll(
        CtreUtil.kMechanismSignalFrequencyHz, m_FlywheelVelocity, m_FlywheelVoltage);

    // Everything else these motors publish is never read here; on CAN FD it all defaults to
    // 100 Hz, so Phoenix decodes it every loop for nothing.
    CtreUtil.reportIfNotOk("Flywheel optimize bus utilization",
        ParentDevice.optimizeBusUtilizationForAll(
            m_flywheelLeadMotor, m_flywheelFollowMotor1, m_flywheelFollowMotor2, m_flywheelFollowMotor3));
  }

  protected final TalonFX m_flywheelLeadMotor = new TalonFX(FlywheelConstants.kLeadMotorCANID, CANBuses.UpperBus);

  protected final TalonFX m_flywheelFollowMotor1 = new TalonFX(FlywheelConstants.kFollowMotor1CANID, CANBuses.UpperBus);

  protected final TalonFX m_flywheelFollowMotor2 = new TalonFX(FlywheelConstants.kFollowMotor2CANID, CANBuses.UpperBus);

  protected final TalonFX m_flywheelFollowMotor3 = new TalonFX(FlywheelConstants.kFollowMotor3CANID, CANBuses.UpperBus);

  protected VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  final StatusSignal<AngularVelocity> m_FlywheelVelocity = m_flywheelLeadMotor.getVelocity();
  final StatusSignal<Voltage> m_FlywheelVoltage = m_flywheelLeadMotor.getMotorVoltage();
  final StatusSignal<Current> m_FlywheelSupCurrent = m_flywheelLeadMotor.getSupplyCurrent();
  final StatusSignal<Current> m_FlywheelStatCurrent = m_flywheelLeadMotor.getStatorCurrent();

  /*
   * Per-motor current, indexed to match FlywheelInputs: [lead, follower1, follower2, follower3].
   * The followers draw the bulk of the flywheel's current and were previously unmeasured; see
   * FlywheelInputs.motorSupplyCurrentAmps.
   */
  private final StatusSignal<Current>[] m_supplyCurrentSignals = supplyCurrentSignals();
  private final StatusSignal<Current>[] m_statorCurrentSignals = statorCurrentSignals();

  /* Every signal updateInputs() refreshes, flattened once here rather than rebuilt at 50 Hz. */
  private final BaseStatusSignal[] m_allSignals = ArrayUtil.concat(
      new BaseStatusSignal[] {m_FlywheelVelocity, m_FlywheelVoltage},
      m_supplyCurrentSignals,
      m_statorCurrentSignals);

  @SuppressWarnings("unchecked")
  private StatusSignal<Current>[] supplyCurrentSignals() {
    return new StatusSignal[] {
        m_FlywheelSupCurrent,
        m_flywheelFollowMotor1.getSupplyCurrent(),
        m_flywheelFollowMotor2.getSupplyCurrent(),
        m_flywheelFollowMotor3.getSupplyCurrent()
    };
  }

  @SuppressWarnings("unchecked")
  private StatusSignal<Current>[] statorCurrentSignals() {
    return new StatusSignal[] {
        m_FlywheelStatCurrent,
        m_flywheelFollowMotor1.getStatorCurrent(),
        m_flywheelFollowMotor2.getStatorCurrent(),
        m_flywheelFollowMotor3.getStatorCurrent()
    };
  }

  protected void configureMotors() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);
    flywheelConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(FlywheelConstants.kStatorCurrentLimitAmps)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(FlywheelConstants.kSupplyCurrentLimitAmps)
        .withSupplyCurrentLimitEnable(true);
    flywheelConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(FlywheelConstants.kReduction);
    m_flywheelFollowMotor1.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    m_flywheelFollowMotor2.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    m_flywheelFollowMotor3.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelConfig.Slot0 = new Slot0Configs()
        .withKP(FlywheelConstants.kP)
        .withKS(FlywheelConstants.kS)
        .withKV(FlywheelConstants.kV);
    flywheelConfig.Voltage = new VoltageConfigs()
        .withPeakForwardVoltage(FlywheelConstants.kNewMaxVolts)
        .withPeakReverseVoltage(FlywheelConstants.kNewMinVolts);
    CtreUtil.reportIfNotOk("Config flywheel (lead)",
        m_flywheelLeadMotor.getConfigurator().apply(flywheelConfig));
    CtreUtil.reportIfNotOk("Config flywheel (follower 1)",
        m_flywheelFollowMotor1.getConfigurator().apply(flywheelConfig));
    CtreUtil.reportIfNotOk("Config flywheel (follower 2)",
        m_flywheelFollowMotor2.getConfigurator().apply(flywheelConfig));
    CtreUtil.reportIfNotOk("Config flywheel (follower 3)",
        m_flywheelFollowMotor3.getConfigurator().apply(flywheelConfig));
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    // One batched CAN round trip for the mechanism signals and all eight current signals.
    BaseStatusSignal.refreshAll(m_allSignals);
    inputs.velocityRPM = m_FlywheelVelocity.getValueAsDouble() * 60;
    inputs.appliedVolts = m_FlywheelVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = m_FlywheelStatCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = m_FlywheelSupCurrent.getValueAsDouble();
    for (int i = 0; i < FlywheelIO.kMotorCount; i++) {
      inputs.motorSupplyCurrentAmps[i] = m_supplyCurrentSignals[i].getValueAsDouble();
      inputs.motorStatorCurrentAmps[i] = m_statorCurrentSignals[i].getValueAsDouble();
    }
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    m_flywheelLeadMotor.setControl(m_velocityRequest.withVelocity(velocityRPM / 60));
  }

  @Override
  public void stop() {
    m_flywheelLeadMotor.stopMotor();
  }

}