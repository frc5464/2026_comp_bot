package frc.robot.subsystems;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.math.geometry.Pose3d;

import org.photonvision.simulation.VisionSystemSim; 

public class Vision {
    /* Main class for vision sub-routine
     * Available functions:
     *  public double getTargetInfoDouble (int fiducialID, String targetField)
     *      Get yaw/pitch/area/skew as specified in targetField. Loops through results
     * 
     *  public Transform3d getTargetInfoPose (int fiducialID)
     *      Uses same method but returns robot pose. Seperate function as it's a seperate return value
     * 
     *  public List<TargetCorner> getTargetInfoCorners (int fiducialID)
     *      See above, gets corners of given AprilField
     * 
     *  public Pose3d robotPose ()
     *      Returns the robot's pose using the modern AprilTag method
     * 
     *  public void visionUpdateLoop()
     *      Automatically called in all mentioned functions, includes call counter for function declaration
     * 
     * 
     *  private AprilTagFieldLayout constructAprilField(String jsonPath)
     *      Primarily for internal declaration and use. safely constructs AprilFields using the passed path (relative)
     */
    // These values should be user-configured as needed
    private PhotonCamera camera = new PhotonCamera("null");
    private final String aprilDataPath = "src\\main\\java\\frc\\robot\\subsystems\\april_tag_layouts\\2026-rebuilt-welded.json";
    AprilTagFieldLayout aprilField = constructAprilField(aprilDataPath);
    
    // These values pull from the aformentioned ones and can stay the same
    private List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    private PhotonPipelineResult result = results.get(results.size()-1);
    private VisionSystemSim visionLayout = new VisionSystemSim("primary");
    
    // TODO: Fill in placeholder values with real values
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
    
    // Apriltag super constructor with try/catch
    private AprilTagFieldLayout constructAprilField(String jsonPath){
        try {
            return new AprilTagFieldLayout(aprilDataPath);
        } 
        catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }
   
    //# of times loopthrough, used for variable declaration
    private int loopCount = 0;

    public void visionUpdateLoop(){
        if (loopCount == 0){
            visionLayout.addAprilTags(aprilField);
        } loopCount++;

        // re-read from camera
        results=camera.getAllUnreadResults();
        result = results.get(results.size()-1);

        // position estimates
        mainPoseEst.update(swerveGyroAngle, swerveModPos);
    }
   
    // Internal function used for getting double values, requires fiducialID and what value is needed (yaw, pitch, area, etc etc)
    public double getTargetInfoDouble (int fiducialID, String targetField) {
        visionUpdateLoop();
        if (!result.hasTargets()){
            return (double) 0;
        }
        for (PhotonTrackedTarget i : result.getTargets()) {
            if (i.getFiducialId() != fiducialID){
                continue;
            }

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

    public Pose3d robotPose (){
        if (!result.hasTargets()){
            return null;
        }
        for (PhotonTrackedTarget i : result.getTargets()) {
            if (aprilField.getTagPose(i.getFiducialId()).isPresent()) {
                return PhotonUtils.estimateFieldToRobotAprilTag(
                    i.getBestCameraToTarget(),
                    aprilField.getTagPose(i.getFiducialId()).get(),
                    getTargetInfoPose(i.getFiducialId()));
            }
        }
        return null;
    }
 
}