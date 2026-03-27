package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static AprilTagFieldLayout FieldSuperConstructor(String path) {
        AprilTagFieldLayout data = null;

        try {
            data = AprilTagFieldLayout.loadFromResource(path);
            return data;
        } catch (Exception e) {
            System.err.println("FieldSuperConstructor failed to load JSON map!");
            SmartDashboard.putBoolean("Loaded ATF", false);
        }
        return null;
    }

    private final static AprilTagFieldLayout kTagFieldLayout = FieldSuperConstructor(
            "src/main/java/frc/robot/subsystems/vision_extra/2026-rebuilt-welded.json");

    private class lookupTable {
        // functions as a dictionary,
        // where the index of a key is used as the index of a result
        // interact with using functions, not directly
        // key/result mismatch may otherwise occur

        List<String> keys = new ArrayList<>();
        List<List<PhotonPipelineResult>> results = new ArrayList<>();

        public int addItem(String Key, List<PhotonPipelineResult> value) {

            if (!keys.contains(Key)) {
                keys.add(Key);
                results.add(value);
            } else {
                results.set(keys.indexOf(Key), value);
            }

            if (keys.size() != results.size()) {
                System.err.println("Size mismatch in lookuptable");
            }

            return keys.indexOf(Key);
        }

        public List<PhotonPipelineResult> retrieveItem(String Key) {
            return results.get(keys.indexOf(Key));
        }

        public void clear() {
            results.clear();
            keys.clear();
        }
    }

    private PhotonCamera[] cameras = {
            new PhotonCamera("combee"),
            new PhotonCamera("beedril"),
            new PhotonCamera("vespiquen")
    };

    private PhotonPoseEstimator estimatePoseFromCamera(PhotonCamera Camera) {
        // TODONE: enable multitag in GUI
        // TODO: get actual offsets for center of robot
        //      update: info in discord, need to convert to the correct format still
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

    private lookupTable cameraTable = new lookupTable();

    public void periodic() {
        // VisionLoop();
    }

    private void VisionLoop() {

        cameraTable.clear(); /* clear table before adding items again */

        for (PhotonCamera c : cameras) {
            if (!c.getAllUnreadResults().isEmpty()) {
                cameraTable.addItem(c.getName(), c.getAllUnreadResults());
            }

        }

        for (String key : cameraTable.keys) {
            if (!cameraTable.retrieveItem(key).isEmpty()) {
                SmartDashboard.putString(key, cameraTable.retrieveItem(key).toString());
            }

        }
    }

    public List<PhotonTrackedTarget> getAllTargets() {
        List<PhotonTrackedTarget> compiledTargets = new ArrayList<>();

        for (PhotonCamera cam : cameras) {
            compiledTargets.addAll(getCameraTargets(cam.getName()));
        }
        return compiledTargets;
    }

    public List<PhotonTrackedTarget> getCameraTargets(String cameraName) {
        List<PhotonTrackedTarget> compiledResults = new ArrayList<>();
        for (PhotonPipelineResult item : cameraTable.retrieveItem(cameraName)) {
            compiledResults.addAll(item.getTargets());
        }
        return compiledResults;
    }

    public Pose3d compiledRobotPose() {
        // uses several cameras to result in one position
        // average positions to reduce offset

        List<EstimatedRobotPose> estimatedPositions = new ArrayList<>();
        for (PhotonCamera c : cameras) {
            for (PhotonPipelineResult item : cameraTable.retrieveItem(c.getName())) {
                Optional<EstimatedRobotPose> constructorObject = estimatePoseFromCamera(c)
                        .estimateCoprocMultiTagPose(item);

                if (constructorObject.isPresent()) {
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }

                // backup estimation method
                constructorObject = estimatePoseFromCamera(c)
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
        // just completePose modified to be offset from turret point
        // TODO: get offset from center to the turret

        final Transform3d turretOffset = new Transform3d();

        return compiledRobotPose().plus(turretOffset);
    }

    public Pose3d tagPose(PhotonTrackedTarget AprilTag) {
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