package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static AprilTagFieldLayout FieldSuperConstructor(String path) {
        AprilTagFieldLayout data = null;

        try {
            data = AprilTagFieldLayout.loadFromResource(path);
            return data;
        } catch (Exception e) {
            // TODO: handle exception
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

            return keys.indexOf(Key);
        }

        public List<PhotonPipelineResult> retrieveItem(String Key) {
            return results.get(keys.indexOf(Key));
        }

        public void popItem(String Key) {
            results.remove(keys.indexOf(Key));
            keys.remove(Key);
        }

        public void clear() {
            results.clear();
            keys.clear();
        }
    }

    private PhotonCamera[] cameras = {
            new PhotonCamera("ribombee"), /* are these camera names allowed? like frc-wise */
            new PhotonCamera("combee"),
            new PhotonCamera("beedril"),
            new PhotonCamera("vespiquen")
    };
    private lookupTable cameraTable = new lookupTable();

    public void periodic() {
        // dont uncomment and push to main till stable
        // VisionLoop();
    }

    public void VisionLoop() {

        cameraTable.clear(); /* clear table before adding items again */

        for (PhotonCamera c : cameras) {
            cameraTable.addItem(c.getName(), c.getAllUnreadResults());
        }

        for (String key : cameraTable.keys) {
            SmartDashboard.putString(key, cameraTable.retrieveItem(key).toString());
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

    private PhotonPoseEstimator estimatePoseFromCamera(PhotonCamera Camera) {
        // TODO: enable multitag in GUI
        // TODO: get actual offsets for center of robot
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

    public Pose3d compiledCenterPoseMM() {
        // uses several cameras to result in one position
        // average positions to reduce offset

        List<EstimatedRobotPose> estimatedPositions = new ArrayList<>();
        Translation3d completeTranslation = new Translation3d();
        for (PhotonCamera c : cameras) {
            for (PhotonPipelineResult item : cameraTable.retrieveItem(c.getName())) {
                Optional<EstimatedRobotPose> constructorObject = estimatePoseFromCamera(c)
                        .estimateCoprocMultiTagPose(item);

                if (constructorObject.isPresent() && !constructorObject.isEmpty()) {
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }

                // backup estimation method
                constructorObject = estimatePoseFromCamera(c)
                        .estimateLowestAmbiguityPose(item);

                if (constructorObject.isPresent() && !constructorObject.isEmpty()) {
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }

                // TODO: handle double failure
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
        //just completePose modified to be offset from turret point
        // TODO: get offset from center to the turret

        final Transform3d turretOffset = new Transform3d();

        return compiledCenterPoseMM().plus(turretOffset);
    }

    private Rotation2d rotationToTag(int fidID, Boolean fromTurret) {
        // fiducial ID of april tag
        // and True if centered on turret,
        // False if from the center of the robot
        // returns a transform from the center to the target tag
        PhotonTrackedTarget locatedFID = null;
        Pose2d tagPose2d;
        Pose2d selfPose2d;

        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == fidID){
                locatedFID = target;
            }
        }
        if (locatedFID == null) {
            return null;
        } else {
            //TODO: get april tag position
        }

        

        if (fromTurret){
            selfPose2d = turretPose().toPose2d();
        } else {
            selfPose2d = compiledCenterPoseMM().toPose2d();
        }

        return PhotonUtils.getYawToPose(selfPose2d, tagPose2d);

    }
}