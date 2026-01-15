package frc.robot.subsystems;
import java.lang.annotation.Target;
import java.util.List;

//import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
    
 /* public static void main(String[] args) {
    final int UPDATES_PER_SECOND = 2400 * 1000;
    PhotonCamera camera = new PhotonCamera("photonvision");
    boolean VisionLoop = true;
    List<TargetCorner> VisCorners = getCorners(camera);
    final long calculatedUPS = (long) 1/UPDATES_PER_SECOND;
    while(VisionLoop){
        try {
            Thread.sleep(calculatedUPS);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }

  } */
    static Transform3d getPose(PhotonCamera targetCamera){
        var result = targetCamera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getBestCameraToTarget();}
    
        static List<TargetCorner> getCorners(PhotonCamera targetCamera){
        var result = targetCamera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getDetectedCorners();}
    
    static double getDoubleValue(String targetString, PhotonCamera targetCamera){
        var result = targetCamera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        switch (targetString) {
            case "yaw":
                return target.getYaw();
            case "pitch":
                return target.getPitch();
            case "area":
                return target.getArea();
            case "skew":
                return target.getSkew();
            default:
                return -256;
        }
    }
  
}
