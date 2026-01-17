package frc.robot.subsystems;
import java.lang.ref.Cleaner;
import java.util.List;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
public class Vision {
    
    private PhotonCamera camera = new PhotonCamera("null");
    private List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    private PhotonPipelineResult result = results.get(results.size()-1);
    private VisionSystemSim visionLayout = new VisionSystemSim("primary");
    
    //private final DifferentialDrivePoseEstimator robotPose = 
    //    new DifferentialDrivePoseEstimator(
    //        
    //    );

    //# of times loopthrough, used for variable declaration
    private int loopCount = 0;
    private void visionUpdateLoop(){
        if (loopCount == 0){
            visionLayout.addAprilTags(null);
            //setup AprilTag layout
        }
        loopCount++;
        
        // re-read from camera
        results=camera.getAllUnreadResults();
        result = results.get(results.size()-1);

    }
    
    public double getTargetInfoDouble (int fiducialID, String targetField) {
        visionUpdateLoop();
        if (result.hasTargets()){
            for (PhotonTrackedTarget i : result.getTargets()) {
                if (i.getFiducialId() == fiducialID){
                    switch (targetField) {
                        case "yaw":
                            return i.getYaw();
                        case "pitch":
                            return i.getPitch();
                        case "area":
                            return i.getArea();
                        case "skew":
                            return i.getSkew();
                        default:
                            continue;
                    }
                }
            }
        }
        // return zero if no AprilTag found
        return (double) 0;
    }

    public void RelativePosition() {
            // TODO: make it work
        return; 
    }
 
}