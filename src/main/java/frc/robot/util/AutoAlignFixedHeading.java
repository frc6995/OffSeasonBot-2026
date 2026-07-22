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

    private Rotation2d m_heading;

    private boolean m_cardinalize = false;

    public AutoAlignFixedHeading(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain, Rotation2d heading) {
        super(targetPose, entryAngle, drivetrain);
        this.m_heading = heading;
    }

    public AutoAlignFixedHeading(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, Rotation2d heading) {
        this(new APTarget(targetPose), drivetrain, heading, AutoAlignConstants.DEFAULT_CONSTRAINTS);
    }

    public AutoAlignFixedHeading(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain, boolean cardinalize) {
        this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain, AutoAlignConstants.DEFAULT_CONSTRAINTS, cardinalize);
    }

    public AutoAlignFixedHeading(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, boolean cardinalize) {
        this(new APTarget(targetPose), drivetrain, AutoAlignConstants.DEFAULT_CONSTRAINTS, cardinalize);
        
    }

    public AutoAlignFixedHeading(APTarget target, CommandSwerveDrivetrain drivetrain, Rotation2d fixedHeading, APConstraints constraints) {
        super(target, drivetrain, constraints);
        this.m_heading = fixedHeading;
    }

    public AutoAlignFixedHeading(APTarget target, CommandSwerveDrivetrain drivetrain, APConstraints constraints, boolean cardinalize) {
        super(target, drivetrain, constraints);
        this.m_cardinalize = cardinalize;
        this.m_heading = drivetrain.state().Pose.getRotation();
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
    public void execute() {
            swerveState = m_drivetrain.getState();
            APResult out;
            if (m_cardinalize) {
                m_realTarget = new APTarget(new Pose2d(
                        m_target.getReference().getTranslation().getX(), m_target.getReference().getY(), 
                                cardinalizeHeading(m_heading)));
                out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_realTarget);
            }
            else {
                m_realTarget = m_target;
                out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);
            }

            applyDriveRequest(out);
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_realTarget);
    }

}
