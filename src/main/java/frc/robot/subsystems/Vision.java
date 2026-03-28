package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.SmartCam;

public class Vision extends SubsystemBase {

    // super constructor was buggy, this is static and just works.
    public static final AprilTagFieldLayout kTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    private boolean initialized = false;

    public class SmartCam {
        String name;
        PhotonCamera obj;
        List<PhotonPipelineResult> results = new ArrayList<>();

        public SmartCam(String cameraName) {
            name = cameraName;
            obj = new PhotonCamera(cameraName);
            
        }

        public void update() {
            results = obj.getAllUnreadResults();
        }

        public Optional<List<PhotonPipelineResult>> SafeResults() {
            Optional<List<PhotonPipelineResult>> optionalResults = Optional.ofNullable(results);
            if (optionalResults.isPresent() && !optionalResults.isEmpty()) {
                if (optionalResults.get().size() == 0){ /*seperate if statement incase empty Optional */
                    return Optional.empty();
                }
                return optionalResults;
            } else {
                return Optional.empty();
            }
            
        }

    }
    
    SmartCam cameras[] = {
        new SmartCam("beedril"),
        new SmartCam("vespiquen"),
        new SmartCam("combee")
    };

    private PhotonPoseEstimator estimatePoseFromCamera(PhotonCamera Camera) {
        // TODONE: enable multitag in GUI
        // TODO: get actual offsets for center of robot
        // update: info in discord, need to convert to the correct format still
        Transform3d relativeCameraPosition = null;
        switch (Camera.getName()) {
            case "ribombee":
                relativeCameraPosition = new Transform3d(new Translation3d(), new Rotation3d());
                break;

            case "combee":
                relativeCameraPosition = new Transform3d(new Translation3d(), new Rotation3d());
                break;

            case "beedril":
                relativeCameraPosition = new Transform3d(new Translation3d(), new Rotation3d());
                break;

            case "vespiquen":
                relativeCameraPosition = new Transform3d(new Translation3d(), new Rotation3d());
                break;

            default:
                SmartDashboard.putBoolean("bad cam", true);
                break;
        }
        return new PhotonPoseEstimator(kTagFieldLayout, relativeCameraPosition);
    }

    

    public void init() {
        SmartDashboard.putBoolean("VisionInit", true);
        initialized = true;
    }

    public void periodic() {

      for (SmartCam c : cameras) {
        c.update();
      }

        if (!initialized) {
            init();
        }
        VisionLoop();
    }

    private void VisionLoop() {
        // tested. working
        //  lol no
        for (SmartCam c : cameras) {
            if (c.SafeResults().isEmpty()){
                continue;
            }
            SmartDashboard.putString(c.name,c.SafeResults().get().toString());
        }
    }

    public List<PhotonTrackedTarget> getAllTargets() {
        // TODO: TEST
        List<PhotonTrackedTarget> compiledTargets = new ArrayList<>();

        for (SmartCam c : cameras) {
            if (c.SafeResults().isEmpty()){
                continue;
            }
            for (PhotonPipelineResult sR : c.SafeResults().get()) {
                compiledTargets.addAll(sR.getTargets());
            }
        }
        return compiledTargets;
    }

    public List<PhotonTrackedTarget> getCameraTargets(String cameraName) {
        // TODO: TEST
        List<PhotonTrackedTarget> compiledResults = new ArrayList<>();
        for (SmartCam c : cameras) {
            if (c.name == cameraName) {
                if (c.SafeResults().isEmpty()){
                    continue;
                }
                for (PhotonPipelineResult sR : c.SafeResults().get()) {
                    compiledResults.addAll(sR.getTargets());
                }
            }
        }

        return compiledResults;
    }

    public Pose3d compiledRobotPose() {
        // TODO: TEST
        // uses several cameras to result in one position
        // average positions to reduce offset

        List<EstimatedRobotPose> estimatedPositions = new ArrayList<>();
        for (SmartCam c : cameras) {
            if (c.SafeResults().isEmpty()){
                continue;
            }
            for (PhotonPipelineResult item : c.SafeResults().get()) {
                Optional<EstimatedRobotPose> constructorObject = estimatePoseFromCamera(c.obj)
                        .estimateCoprocMultiTagPose(item);

                if (constructorObject.isPresent()) {
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }

                // backup estimation method
                constructorObject = estimatePoseFromCamera(c.obj)
                        .estimateLowestAmbiguityPose(item);

                if (constructorObject.isPresent()) {
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }
                System.err.println("Both pose estimation types failed to find any tags.");
                SmartDashboard.putBoolean("failedEstimation", true);
            }
        }

        // return mean of all estimations
        Translation3d avgTranslation = new Translation3d(); // seperate into translation and rotation
        Rotation3d avgRotation = new Rotation3d();
        for (EstimatedRobotPose estPos : estimatedPositions) {
            avgTranslation = avgTranslation.plus(estPos.estimatedPose.getTranslation()); // add all together seperately
            avgRotation = avgRotation.plus(estPos.estimatedPose.getRotation());
        }
        Pose3d completePose = new Pose3d(avgTranslation, avgRotation); // recombine
        completePose = completePose.div(estimatedPositions.size()); // divide by len
        return completePose;
    }

    public Pose3d turretPose() {
        // TODO: TEST
        // just completePose modified to be offset from turret point
        // TODO: get offset from center to the turret

        final Transform3d turretOffset = new Transform3d();

        return compiledRobotPose().plus(turretOffset);
    }

    public Pose3d tagPose(PhotonTrackedTarget AprilTag) {
        // TODO: TEST
        return new Pose3d(
                new Translation3d(AprilTag.bestCameraToTarget.getX(), AprilTag.bestCameraToTarget.getY(),
                        AprilTag.bestCameraToTarget.getZ()),
                new Rotation3d(AprilTag.skew, AprilTag.pitch, AprilTag.yaw));
    }

    public Rotation2d rotationToTag(int fidID, Boolean fromTurret) {
        // fiducial ID of april tag
        // and True if centered on turret,
        // False if from the center of the robot
        // returns a transform from the center to the target tag

        // TODO: TEST

        PhotonTrackedTarget locatedFID = null;
        Pose3d tagPose3d;
        Pose3d selfPose3d;

        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == fidID) {
                locatedFID = target;
            }
        }
        if (locatedFID == null) {
            return null;
        } else {
            tagPose3d = tagPose(locatedFID);
        }

        if (fromTurret) {
            selfPose3d = turretPose();
        } else {
            selfPose3d = compiledRobotPose();
        }

        return PhotonUtils.getYawToPose(selfPose3d.toPose2d(), tagPose3d.toPose2d());

    }
}