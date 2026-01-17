package frc.robot.subsystems;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.simulation.VisionSystemSim; 

public class Vision {
    // these values should be user-configured as needed
    private PhotonCamera camera = new PhotonCamera("null");
    private String aprilTagPath = "april_tag_layouts\\2026-rebuilt-welded.json";
    // these values pull from the aformentioned ones and can be effectively static
    private List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    private PhotonPipelineResult result = results.get(results.size()-1);
    private VisionSystemSim visionLayout = new VisionSystemSim("primary");
    

    // TODO: Fill in real values
    private SwerveDriveKinematics swerveDriveKin;
    private Rotation2d swerveGyroAngle;
    private SwerveModulePosition[] swerveModPos;
    private Pose2d swerveInitPos;
    private Matrix<N3, N1> swerveStdDev;
    private Matrix<N3, N1> swerveVisMeasurementStdDev;


    private final SwerveDrivePoseEstimator mainPoseEst = new SwerveDrivePoseEstimator(
        swerveDriveKin,
        swerveGyroAngle,
        swerveModPos,
        swerveInitPos, 
        swerveStdDev, 
        swerveVisMeasurementStdDev
        
    );
    

    //# of times loopthrough, used for variable declaration
    private int loopCount = 0;
    private void visionUpdateLoop(){
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

        // position estimates
        mainPoseEst.update(swerveGyroAngle, swerveModPos);
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
        return ((double) 0);
    }

    public Transform3d getTargetInfoPose (int fiducialID) {
        visionUpdateLoop();
        for (PhotonTrackedTarget i : result.getTargets()) {
            if (i.getFiducialId() == fiducialID){
                return i.getBestCameraToTarget();
            }    
        }
        return (null);
    }

    public List<TargetCorner> getTargetInfoCorners (int fiducialID){
        visionUpdateLoop();
        for (PhotonTrackedTarget i : result.getTargets()) {
            if (i.getFiducialId() == fiducialID){
                return i.getDetectedCorners();
            }
        }
        return (null);
    }

 
}