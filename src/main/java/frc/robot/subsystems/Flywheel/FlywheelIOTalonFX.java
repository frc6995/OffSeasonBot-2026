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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelConstants;
import frc.robot.Constants.CANBuses;

public class FlywheelIOTalonFX implements FlywheelIO {

  protected final TalonFX m_flywheelLeadMotor = new TalonFX(FlywheelConstants.kLeadMotorCANID, CANBuses.UpperBus);

  protected final TalonFX m_flywheelFollowMotor1 = new TalonFX(FlywheelConstants.kFollowMotor1CANID, CANBuses.UpperBus);

  protected final TalonFX m_flywheelFollowMotor2 = new TalonFX(FlywheelConstants.kFollowMotor2CANID, CANBuses.UpperBus);

  protected final TalonFX m_flywheelFollowMotor3 = new TalonFX(FlywheelConstants.kFollowMotor3CANID, CANBuses.UpperBus);
  
  protected VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  
  final StatusSignal<AngularVelocity> m_FlywheelVelocity = m_flywheelLeadMotor.getVelocity();
  final StatusSignal<Voltage> m_FlywheelVoltage = m_flywheelLeadMotor.getMotorVoltage();
  final StatusSignal<Current> m_FlywheelSupCurrent = m_flywheelLeadMotor.getSupplyCurrent();
  final StatusSignal<Current> m_FlywheelStatCurrent = m_flywheelLeadMotor.getStatorCurrent();

  protected void configureMotors() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    flywheelConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FlywheelConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(FlywheelConstants.kSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);
    flywheelConfig.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(FlywheelConstants.kReduction);
          m_flywheelFollowMotor1.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
          m_flywheelFollowMotor2.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
          m_flywheelFollowMotor3.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelConfig.Slot0 =
        new Slot0Configs()
            .withKP(FlywheelConstants.kP)
            .withKS(FlywheelConstants.kS)
            .withKV(FlywheelConstants.kV);
    flywheelConfig.Voltage = 
        new VoltageConfigs()
          .withPeakForwardVoltage(FlywheelConstants.kNewMaxVoltage)
          .withPeakReverseVoltage(FlywheelConstants.kNewMinVoltage);
    // CtreUtil.reportIfNotOk("configure example", m_exMotor.getConfigurator().apply(config));
     m_flywheelLeadMotor.getConfigurator().apply(flywheelConfig);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_FlywheelVelocity, m_FlywheelVoltage, m_FlywheelSupCurrent, m_FlywheelStatCurrent);
    inputs.velocityRPM =
        (m_flywheelLeadMotor.getVelocity().refresh().getValueAsDouble()*60);
    inputs.appliedVolts = m_flywheelLeadMotor.getMotorVoltage().refresh().getValueAsDouble();
    inputs.statorCurrentAmps = m_flywheelLeadMotor.getStatorCurrent().refresh().getValueAsDouble();
    inputs.supplyCurrentAmps = m_flywheelLeadMotor.getSupplyCurrent().refresh().getValueAsDouble();
    inputs.leadMotorConnected = m_flywheelLeadMotor.isConnected();
    inputs.followerMotor1Connected = m_flywheelFollowMotor1.isConnected();
    inputs.followerMotor2Connected = m_flywheelFollowMotor2.isConnected();
    inputs.followerMotor3Connected = m_flywheelFollowMotor3.isConnected();
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
  m_flywheelLeadMotor.setControl(m_velocityRequest.withVelocity(velocityRPM/60));
  }

  @Override
  public void stop() {
    m_flywheelLeadMotor.stopMotor();
  }

}