package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  public static void main(String[] args) {
    PhotonCamera camera = new PhotonCamera("photonvision");
    var result = camera.getLatestResult();
    boolean DoLoop = true;
    while (DoLoop){
        DoLoop = false;

    }
  }
}
