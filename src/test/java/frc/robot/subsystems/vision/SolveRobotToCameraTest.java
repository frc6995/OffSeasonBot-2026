package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.turret.Turret.TurretConstants;

/**
 * Checks {@link ATVision#solveRobotToCamera(double)} against hand-written trig, with no WPILib
 * geometry on the expected side. If these pass, the camera orbits the turret pivot and its lens
 * points radially outward (tilted by the mount pitch) at every turret angle.
 */
class SolveRobotToCameraTest {
    private static final double kEps = 1e-9;

    @Test
    void cameraOrbitsPivotAndFacesOutward() {
        Translation3d pivot = TurretConstants.turretCenterPose.getTranslation();
        double radius = TurretConstants.CAMERA_POSE3D.getX();
        double pitch = TurretConstants.CAMERA_POSE3D.getRotation().getY();

        for (int deg = -360; deg <= 360; deg += 15) {
            double yaw = Math.toRadians(deg);
            Pose3d cam = ATVision.solveRobotToCamera(deg);

            // Position: pivot + radius along the turret's facing direction.
            assertEquals(pivot.getX() + radius * Math.cos(yaw), cam.getX(), kEps, "x at " + deg);
            assertEquals(pivot.getY() + radius * Math.sin(yaw), cam.getY(), kEps, "y at " + deg);
            assertEquals(pivot.getZ(), cam.getZ(), kEps, "z at " + deg);

            // Lens direction: the camera's +X axis expressed in the robot frame.
            Translation3d forward = new Translation3d(1, 0, 0).rotateBy(cam.getRotation());
            assertEquals(Math.cos(pitch) * Math.cos(yaw), forward.getX(), kEps, "fwd x at " + deg);
            assertEquals(Math.cos(pitch) * Math.sin(yaw), forward.getY(), kEps, "fwd y at " + deg);
            assertEquals(-Math.sin(pitch), forward.getZ(), kEps, "fwd z at " + deg);

            // Euler angles actually sent to the Limelight.
            assertEquals(0.0, cam.getRotation().getX(), kEps, "roll at " + deg);
            assertEquals(pitch, cam.getRotation().getY(), kEps, "pitch at " + deg);
            assertEquals(Math.IEEEremainder(yaw, 2 * Math.PI),
                Math.IEEEremainder(cam.getRotation().getZ(), 2 * Math.PI), kEps, "yaw at " + deg);
        }
    }
}
