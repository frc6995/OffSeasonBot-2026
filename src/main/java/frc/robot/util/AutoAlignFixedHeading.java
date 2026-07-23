package frc.robot.util;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignFixedHeading extends AutoAlign {
    private APTarget m_realTarget;

    private final Rotation2d m_heading;
    private final boolean m_cardinalize;

    public AutoAlignFixedHeading(
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            Rotation2d heading,
            RotationControlMode rotationControlMode) {
        super(
                targetPose,
                entryAngle,
                drivetrain,
                kDefaultProfile,
                rotationControlMode,
                AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
        m_heading = heading;
        m_cardinalize = false;
    }

    public AutoAlignFixedHeading(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Rotation2d heading,
            RotationControlMode rotationControlMode) {
        this(new APTarget(targetPose), drivetrain, heading, AutoAlignConstants.DEFAULT_CONSTRAINTS, rotationControlMode);
    }

    public AutoAlignFixedHeading(
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            boolean cardinalize,
            RotationControlMode rotationControlMode) {
        this(
                new APTarget(targetPose).withEntryAngle(entryAngle),
                drivetrain,
                AutoAlignConstants.DEFAULT_CONSTRAINTS,
                cardinalize,
                rotationControlMode);
    }

    public AutoAlignFixedHeading(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            boolean cardinalize,
            RotationControlMode rotationControlMode) {
        this(new APTarget(targetPose), drivetrain, AutoAlignConstants.DEFAULT_CONSTRAINTS, cardinalize, rotationControlMode);
    }

    public AutoAlignFixedHeading(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            Rotation2d fixedHeading,
            APConstraints constraints,
            RotationControlMode rotationControlMode) {
        super(target, drivetrain, constraints, rotationControlMode, AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
        m_heading = fixedHeading;
        m_cardinalize = false;
    }

    public AutoAlignFixedHeading(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APConstraints constraints,
            boolean cardinalize,
            RotationControlMode rotationControlMode) {
        super(target, drivetrain, constraints, rotationControlMode, AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
        m_cardinalize = cardinalize;
        m_heading = drivetrain.state().Pose.getRotation();
    }

    public static Rotation2d cardinalizeHeading(Rotation2d heading) {
        double hdegrees = MathUtil.inputModulus(heading.getDegrees(), -180, 180);
        if (hdegrees >= -135 && hdegrees < -45) {
            return Rotation2d.fromDegrees(-90);
        }
        else if (hdegrees >= -45 && hdegrees < 45) {
            return Rotation2d.kZero;
        }
        else if (hdegrees >= 45 && hdegrees < 135) {
            return Rotation2d.fromDegrees(90);
        }
        else if (hdegrees >= 135 && hdegrees < 180) {
            return Rotation2d.k180deg;
        }
        else if (hdegrees >= -180 && hdegrees < -135) {
            return Rotation2d.fromDegrees(-180);
        }
        else {
            return Rotation2d.kZero;
        }
    }

    public static Rotation2d cardinalizeHeadingNS(Rotation2d heading) {
        double hdegrees = MathUtil.inputModulus(heading.getDegrees(), -180, 180);
        if (hdegrees >= -90 && hdegrees <= 90) {
            return Rotation2d.kZero;
        }
        else if (hdegrees > 90 && hdegrees <= 180) {
            return Rotation2d.k180deg;
        }
        else if (hdegrees < -90 && hdegrees >= -180) {
            return Rotation2d.fromDegrees(-180);
        }
        else {
            return Rotation2d.kZero;
        }
    }

    @Override
    public void initialize() {
        super.initialize();

        Rotation2d targetHeading = m_cardinalize ? cardinalizeHeading(m_heading) : m_heading;
        Pose2d targetPose = new Pose2d(m_target.getReference().getTranslation(), targetHeading);

        m_realTarget = m_target.withReference(targetPose);
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_realTarget);

        applyDriveRequest(out);
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_realTarget);
    }
}
