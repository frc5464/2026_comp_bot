package frc.robot.subsystems;
import java.util.List;

//import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {

    PhotonCamera camera = new PhotonCamera("null");
    private List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    private PhotonPipelineResult result = results.get(results.size()-1);

    private void visionUpdateLoop(){
        results=camera.getAllUnreadResults();
        result = results.get(results.size()-1);

    }
    //private PhotonCamera camera = new PhotonCamera("default");
    
    public double getTargetInfoDouble (int fidicalID, String targetField) {
        visionUpdateLoop();
        if (result.hasTargets()){
            for (PhotonTrackedTarget i : result.getTargets()) {
                if (i.getFiducialId() == fidicalID){
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
 
}