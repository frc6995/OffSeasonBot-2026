package frc.robot.subsystems.Flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Flywheel.RealFlywheel.FlywheelConstants;
// import frc.robot.util.CtreUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

  protected final TalonFX m_flywheelLeadMotor = new TalonFX(FlywheelConstants.kLeadMotorCANID, FlywheelConstants.kHigherBus);

  protected final TalonFX m_flywheelFollowMotor1 = new TalonFX(FlywheelConstants.kFollowMotor1CANID,FlywheelConstants.kHigherBus);

  protected final TalonFX m_flywheelFollowMotor2 = new TalonFX(FlywheelConstants.kFollowMotor2CANID,FlywheelConstants.kHigherBus);

  protected final TalonFX m_flywheelFollowMotor3 = new TalonFX(FlywheelConstants.kFollowMotor1CANID,FlywheelConstants.kHigherBus);
  
  protected VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  protected VoltageOut m_VoltageOut = new VoltageOut(0);

  protected void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    config.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FlywheelConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(FlywheelConstants.kSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);
    config.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(FlywheelConstants.kReduction);
          m_flywheelFollowMotor1.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
          m_flywheelFollowMotor2.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
          m_flywheelFollowMotor3.setControl(new Follower(m_flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    config.Slot0 =
        new Slot0Configs()
            .withKP(FlywheelConstants.kP)
            .withKV(FlywheelConstants.kV);
    // CtreUtil.reportIfNotOk("configure example", m_exMotor.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
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
  m_flywheelLeadMotor.setControl(m_velocityRequest.withVelocity(velocityRPM));
  }

  @Override
  public void stop() {
    m_flywheelLeadMotor.stopMotor();
  }

}