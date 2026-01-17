package frc.robot.subsystems;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import org.photonvision.simulation.VisionSystemSim;
public class Vision {
    // these values should be user-configured as needed
    private PhotonCamera camera = new PhotonCamera("null");
    private String aprilTagPath = "april_tag_layouts\\2026-rebuilt-welded.json";
    
    // these values pull from the aformentioned ones and can be effectively static
    private List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    private PhotonPipelineResult result = results.get(results.size()-1);
    private VisionSystemSim visionLayout = new VisionSystemSim("primary");
    private SwerveDrivePoseEstimator swerveDriveMainPose;
    

    //# of times loopthrough, used for variable declaration
    private int loopCount = 0;
    private void visionUpdateLoop(){
        swerveDriveMainPose.update(null, null);
        if (loopCount == 0){
            // try/catch incase of invalid path
            try {
                visionLayout.addAprilTags(new AprilTagFieldLayout(aprilTagPath));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        loopCount++;

        // re-read from camera
        results=camera.getAllUnreadResults();
        result = results.get(results.size()-1);

    }
    // internal function used for getting double values, requires fiducialID and what value is needed (yaw, pitch, area, etc etc)
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