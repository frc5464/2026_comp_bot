package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

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
     *  public double aprilTagDistance(double atMD, int fiducialID)
     *      return distance to AT as double (0-1), with 1 being 100% of the screen filled.
     *      offset by atMD (april tag minimum distance)
     * 
     *  private AprilTagFieldLayout ATFLsuperConstructor()
     *      returns AprilTagFieldLayout data from jsonpath
     */

    // These values should be user-configured as needed
    private PhotonCamera[] cameras = {
        new PhotonCamera("apis"), /*shooter-side*/
        new PhotonCamera("crabro") /*other-side*/
    };
    
    private static final String jsonPath = "C:\\Users\\cummi\\Documents\\2026 Code\\2026_comp_bot\\src\\main\\java\\frc\\robot\\subsystems\\vision_extra\\2026-rebuilt-andymark.json";
    private static final boolean enableDebugOutput = true;
    private AprilTagFieldLayout ATFLsuperConstructor(){
        try {
            return (new AprilTagFieldLayout(jsonPath));
        } catch (IOException e) {
            // TOD0 Auto-generated catch block
            e.printStackTrace();
        }
        // err out
        return (new AprilTagFieldLayout(null, (double) 0, (int) 1));
    }
    
    public final AprilTagFieldLayout kTagLayout = ATFLsuperConstructor();
    
    private List<PhotonPipelineResult> results;
    private VisionSystemSim visionLayout = new VisionSystemSim("primary");
    private boolean targetful = false;
    private List<PhotonTrackedTarget> cuTrackedTargets;
    // TODO: Fill in placeholder values with real values
    private SwerveDriveKinematics swerveDriveKin;
    private Rotation2d swerveGyroAngle; 
    private SwerveModulePosition[] swerveModPos; // in getStateInfo
    private Pose2d swerveInitPos;
    private Matrix<N3, N1> swerveStdDev;
    private Matrix<N3, N1> swerveVisMeasurementStdDev;

    public void getStateInfo(SwerveDriveState state){
        swerveModPos = state.ModulePositions;
    }

    public SwerveDrivePoseEstimator mainPoseEst = new SwerveDrivePoseEstimator(
        swerveDriveKin,
        swerveGyroAngle,
        swerveModPos,
        swerveInitPos, 
        swerveStdDev, 
        swerveVisMeasurementStdDev
    );
   
    //# of times loopthrough, used for variable declaration
    private int loopCount = 0;

    public void visionUpdateLoop(){
        results.clear();
        targetful = false;
        cuTrackedTargets.clear();
        if (loopCount == 0){
            visionLayout.addAprilTags(kTagLayout);
        } loopCount++;
        for (PhotonCamera c : cameras) {
            results.addAll(c.getAllUnreadResults());
        }
        for (PhotonPipelineResult r : results) {
            if (r.hasTargets()){
                targetful = true;
                break;
            }else{
                continue;
            }
                    
        }
        if (targetful){
            for (PhotonPipelineResult r : results) {
                results.add(r);
                cuTrackedTargets.add(r.getBestTarget());
                
            }
        if (enableDebugOutput){
            SmartDashboard.putBoolean("has_targets", targetful);
            SmartDashboard.putNumber("loop_count", loopCount);
        }
    }


        
        // position estimates
        mainPoseEst.update(swerveGyroAngle, swerveModPos);
    }
   
    // Internal function used for getting double values, requires fiducialID and what value is needed (yaw, pitch, area, etc etc)
    public double getTargetInfoDouble (int fiducialID, String targetField) {
        visionUpdateLoop();
        if (!targetful){
            return (double) 0;
        }
        for (PhotonTrackedTarget i : cuTrackedTargets) {
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
        for (PhotonTrackedTarget i : cuTrackedTargets) {
            if (i.getFiducialId() == fiducialID){
                return i.getBestCameraToTarget();
            }    
        }
        return (null);
    }

    public List<TargetCorner> getTargetInfoCorners (int fiducialID){
        visionUpdateLoop();
        for (PhotonTrackedTarget i : cuTrackedTargets) {
            if (i.getFiducialId() == fiducialID){
                return i.getDetectedCorners();
            }
        }
        return (null);
    }

    public Pose3d robotPose (){
        visionUpdateLoop();
        if (!targetful){
            return null;
        }
        for (PhotonTrackedTarget i : cuTrackedTargets) {
            if (kTagLayout.getTagPose(i.getFiducialId()).isPresent()) {
                return PhotonUtils.estimateFieldToRobotAprilTag(
                    i.getBestCameraToTarget(),
                    kTagLayout.getTagPose(i.getFiducialId()).get(),
                    getTargetInfoPose(i.getFiducialId()));
            }
        }
        return null;
    }

    public double aprilTagDistance(double atMD, int fiducialID){
        //returns 0.0 if april tag not found
        //returns 1.0 if april tag area >= atMS

        final double ATArea = getTargetInfoDouble(fiducialID, "area")*0.01;

        if (ATArea == 0){
            return (double) 0;
        }

        if (ATArea >= 1-atMD){
            return (double) 1;
        } else{
            return ATArea;
        }
    }

    public void rotateToTag(int fiducialId) {
        visionUpdateLoop();
        if (targetful) {
            
            //private turn = -1.0*getTargetInfoDouble(fiducialId, "yaw")*Constants.kMaxTurnRateDegPerS
        }
    }
}
