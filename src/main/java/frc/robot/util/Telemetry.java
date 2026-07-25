package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;


import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;


public class Telemetry {

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final double[] m_poseArray = new double[3];

    /**
     * Accept the swerve drive state and telemeterize it to SmartDashboard and
     * SignalLogger.
     */
    public void telemeterize(SwerveDriveState state) {
        // /* Telemeterize the swerve drive state */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);

        // /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

    }
}
