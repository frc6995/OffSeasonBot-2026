package frc.robot.subsystems.example;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.example.Example.ExampleConstants;
import frc.robot.util.CtreUtil;

public class ExampleIOTalonFX implements ExampleIO {

    protected final TalonFX m_intakePivotMotor =
            new TalonFX(ExampleConstants.kINTAKE_PIVOT_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_intakeRollerMotor =
            new TalonFX(ExampleConstants.kINTAKE_ROLLER_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final MotionMagicVoltage m_intakePivotRequest =
            new MotionMagicVoltage(0.0).withEnableFOC(true);
    protected final VelocityVoltage m_intakeRollerRequest =
            new VelocityVoltage(0.0).withEnableFOC(true);

    private final StatusSignal<Angle> m_intakePivotPosition = m_intakePivotMotor.getPosition();
    private final StatusSignal<Voltage> m_intakePivotAppliedVoltage =
            m_intakePivotMotor.getMotorVoltage();
    private final StatusSignal<Current> m_intakePivotStatorCurrent =
            m_intakePivotMotor.getStatorCurrent();
    private final StatusSignal<Current> m_intakePivotSupplyCurrent =
            m_intakePivotMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_intakeRollerVelocity =
            m_intakeRollerMotor.getVelocity();
    private final StatusSignal<Voltage> m_intakeRollerAppliedVoltage =
            m_intakeRollerMotor.getMotorVoltage();
    private final StatusSignal<Current> m_intakeRollerStatorCurrent =
            m_intakeRollerMotor.getStatorCurrent();
    private final StatusSignal<Current> m_intakeRollerSupplyCurrent =
            m_intakeRollerMotor.getSupplyCurrent();

    public ExampleIOTalonFX() {
        configureMotors();
    }

    protected void configureMotors() {
        configureIntakePivotMotors();
        configureIntakeRollerMotors();
    }

    private void configureIntakePivotMotors() {
        TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration();
        intakePivotConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        intakePivotConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ExampleConstants.kIntakePivotStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ExampleConstants.kIntakePivotSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        intakePivotConfig.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(ExampleConstants.kIntakePivotReduction);
        intakePivotConfig.Slot0 = new Slot0Configs()
                .withKP(ExampleConstants.kIntakePivotP)
                .withKS(ExampleConstants.kIntakePivotS)
                .withKV(ExampleConstants.kIntakePivotV)
                .withKG(ExampleConstants.kIntakePivotG)
                .withGravityType(GravityTypeValue.Arm_Cosine);
        intakePivotConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ExampleConstants.kIntakePivotCruiseVelocity)
                .withMotionMagicAcceleration(ExampleConstants.kIntakePivotAcceleration);

        CtreUtil.reportIfNotOk(
                "configure intakePivot",
                m_intakePivotMotor.getConfigurator().apply(intakePivotConfig));
    }

    private void configureIntakeRollerMotors() {
        TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
        intakeRollerConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        intakeRollerConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ExampleConstants.kIntakeRollerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ExampleConstants.kIntakeRollerSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        intakeRollerConfig.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(ExampleConstants.kIntakeRollerReduction);
        intakeRollerConfig.Slot0 = new Slot0Configs()
                .withKP(ExampleConstants.kIntakeRollerP)
                .withKS(ExampleConstants.kIntakeRollerS)
                .withKV(ExampleConstants.kIntakeRollerV);

        CtreUtil.reportIfNotOk(
                "configure intakeRoller",
                m_intakeRollerMotor.getConfigurator().apply(intakeRollerConfig));
    }

    @Override
    public void updateInputs(ExampleInputs inputs) {
        inputs.intakePivotMotorConnected =
                BaseStatusSignal.refreshAll(
                        m_intakePivotPosition,
                        m_intakePivotAppliedVoltage,
                        m_intakePivotStatorCurrent,
                        m_intakePivotSupplyCurrent)
                        .isOK();

        inputs.intakePivotPositionDegrees =
                intakePivotRotationsToDegrees(m_intakePivotPosition.getValueAsDouble());
        inputs.intakePivotAppliedVolts = m_intakePivotAppliedVoltage.getValueAsDouble();
        inputs.intakePivotStatorCurrentAmps = m_intakePivotStatorCurrent.getValueAsDouble();
        inputs.intakePivotSupplyCurrentAmps = m_intakePivotSupplyCurrent.getValueAsDouble();

        inputs.intakeRollerMotorConnected =
                BaseStatusSignal.refreshAll(
                        m_intakeRollerVelocity,
                        m_intakeRollerAppliedVoltage,
                        m_intakeRollerStatorCurrent,
                        m_intakeRollerSupplyCurrent)
                        .isOK();

        inputs.intakeRollerVelocityRPM = m_intakeRollerVelocity.getValueAsDouble() * 60.0;
        inputs.intakeRollerAppliedVolts = m_intakeRollerAppliedVoltage.getValueAsDouble();
        inputs.intakeRollerStatorCurrentAmps = m_intakeRollerStatorCurrent.getValueAsDouble();
        inputs.intakeRollerSupplyCurrentAmps = m_intakeRollerSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setIntakePivotPosition(double positionDegrees) {
        m_intakePivotMotor.setControl(
                m_intakePivotRequest.withPosition(intakePivotDegreesToRotations(positionDegrees)));
    }

    @Override
    public void setIntakeRollerVelocity(double velocityRPM) {
        m_intakeRollerMotor.setControl(m_intakeRollerRequest.withVelocity(velocityRPM / 60.0));
    }

    @Override
    public void resetEncoder() {
        m_intakePivotMotor.setPosition(0.0);
    }

    @Override
    public void stop() {
        m_intakePivotMotor.stopMotor();
        m_intakeRollerMotor.stopMotor();
    }

    protected static double intakePivotDegreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    protected static double intakePivotRotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }
}
