//frc
package frc.robot.subsystems;
//java
import java.io.IOException;
import java.util.List;
//photon
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
//wpi
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*vision docs;
 * 
 * finding robot position:
 *  using april tags- Pose3d robotPose()
 *  (WIP) using swerve drive + april tags- mainPoseEst 
 * 
 * getting apriltag info:
 *  yaw/pitch/area/skew:
 *      double getTargetInfoDouble(int fiducialID, String targetField)
 *  distance(relative):
 *      double aprilTagDistance(double atMD, int fiducialID)
 *  camera transform to target:
 *      Transform3d getTargetInfoPose(int fiducialID)
 *  corners of target:
 *      List<TargetCorner> getTargetInfoCorners(int fiducialID)
 * 
 */

public class Vision {
    private PhotonCamera[] cameras = {
            new PhotonCamera("apis"), /* shooter-side */
            new PhotonCamera("crabro") /* other-side */
    };

    private static final String jsonPath = "C:\\Users\\cummi\\Documents\\2026 Code\\2026_comp_bot\\src\\main\\java\\frc\\robot\\subsystems\\vision_extra\\2026-rebuilt-andymark.json";
    private static final boolean enableDebugOutput = true;

    private AprilTagFieldLayout ATFLsuperConstructor() {
        // used to catch errors on file read/write for AprilTag Field Layouts
        try {
            return (new AprilTagFieldLayout(jsonPath));
        } catch (IOException e) {
            e.printStackTrace();
            return (new AprilTagFieldLayout(null, (double) 0, (int) 1));
        }

    }

    public final AprilTagFieldLayout kTagLayout = ATFLsuperConstructor();
    
    private String debugOutputRobotPose3d = robotPose().toString();
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
        new Rotation2d(),
        swerveModPos,

        new Pose2d(2, 4, Rotation2d.fromDegrees(180))); /*PLACEHOLDER VALUES */

    // only updated once, used for defining the AprilTag layout
    private boolean visionLayoutDefined = false;

    public void visionUpdateLoop() {
        // update result list, find targets, and update position estimates
        targetful = false;
        
        if (visionLayoutDefined == false) {
            visionLayout.addAprilTags(kTagLayout);
            visionLayoutDefined = true;
        }

        for (PhotonCamera c : cameras) {
            results=(c.getAllUnreadResults());
        }

        
        if (cuTrackedTargets.size() > 20) {
            cuTrackedTargets.remove(cuTrackedTargets.size()-1);
        }

        for (PhotonPipelineResult r : results) {
            if (r.hasTargets()) {
                targetful = true;
                break;
            } else {
                continue;
            }

        }
        if (targetful) {
            for (PhotonPipelineResult r : results) {
                results.add(r);
                cuTrackedTargets.add(r.getBestTarget());

            }
            if (enableDebugOutput) {
                SmartDashboard.putBoolean("has_targets", targetful);
                SmartDashboard.putString("robot_position", debugOutputRobotPose3d);
            }
        }
        // position estimates
        mainPoseEst.update(swerveGyroAngle, swerveModPos);
    }

    public double getTargetInfoDouble(int fiducialID, String targetField) {
        // returns targetField(yaw,pitch,area,skew) of fiducialID AprilTag
        visionUpdateLoop();
        if (!targetful) {
            return (double) 0;
        }
        for (PhotonTrackedTarget i : cuTrackedTargets) {
            if (i.getFiducialId() != fiducialID) {
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

    public Transform3d getTargetInfoPose(int fiducialID) {
        // gets Transform3d of fiducialID AprilTag
        visionUpdateLoop();
        for (PhotonTrackedTarget i : cuTrackedTargets) {
            if (i.getFiducialId() == fiducialID) {
                return i.getBestCameraToTarget();
            }
        }
        return (null);
    }

    public List<TargetCorner> getTargetInfoCorners(int fiducialID) {
        // get corners of fiducialID AprilTag
        visionUpdateLoop();
        for (PhotonTrackedTarget i : cuTrackedTargets) {
            if (i.getFiducialId() == fiducialID) {
                return i.getDetectedCorners();
            }
        }
        return (null);
    }

    public Pose3d robotPose() {
        // gets pose3d of robot based off of AprilTag positions
        visionUpdateLoop();
        if (!targetful) {
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

    public double aprilTagDistance(double atMD, int fiducialID) {
        // returns 0.0 if april tag not found
        // returns 1.0 if april tag area >= atMS

        final double ATArea = getTargetInfoDouble(fiducialID, "area") * 0.01;

        if (ATArea == 0) {
            return (double) 0;
        }

        if (ATArea >= 1 - atMD) {
            return (double) 1;
        } else {
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
