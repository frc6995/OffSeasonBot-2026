package frc.robot.subsystems.vision.apriltag.photonvision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RealPhotonATVision extends PhotonATVision {
    private PhotonCamera[] m_cameras = new PhotonCamera[PhotonATVisionConstants.PHOTON_IDS.length];
    private PhotonPoseEstimator[] m_estimators = new PhotonPoseEstimator[m_cameras.length];

    public RealPhotonATVision() {
        for(int i = 0; i < m_cameras.length; i++) {
            m_cameras[i] = new PhotonCamera(PhotonATVisionConstants.PHOTON_IDS[i]);
            m_estimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), PhotonATVisionConstants.PHOTON_OFFSETS[i]);
        }
    }

    @Override
    public void periodic() {
        var results = m_cameras[0].getAllUnreadResults();

        for(int i = 1; i < m_cameras.length; i++) {
            results.addAll(m_cameras[i].getAllUnreadResults());
        }

        for(var result : results) {
            
        }
        
    }

    @Override
    public List<PhotonPipelineResult> getAllEstimates() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAllEstimates'");
    }
    
}
