package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.text.DecimalFormat;
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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    // super constructor was buggy, this is static and just works.
    public static final AprilTagFieldLayout kTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    public final static int MIN_POS_SAMPLE = 1;

    public final static int DECI_ACC = 6;
    public int failed_samples, multitag_samples, basic_samples = 0;

    public double vpos_x, vpos_y, vpos_z = 0;
    public Field2d vision_robot_pose = new Field2d();
    public Rotation3d vpos_rot = new Rotation3d();

    public class SmartCam {
        String name;
        PhotonCamera obj;
        List<PhotonPipelineResult> results = new ArrayList<>();

        public SmartCam(String cameraName) {
            name = cameraName;
            obj = new PhotonCamera(cameraName);

        }

        public void update() {
            List<PhotonPipelineResult> cacheResults = obj.getAllUnreadResults();
            if (!cacheResults.isEmpty()) {
                results = cacheResults;
            }
            SmartDashboard.putString(this.name, this.SafeResults().toString());
        }

        public Optional<List<PhotonPipelineResult>> SafeResults() {
            if (results.size() == 0 || results == null) {
                return Optional.empty();
            } else {
                return Optional.of(results);
            }
        }

        public boolean SafeResSafe() {
            if (SafeResults().isEmpty() || SafeResults().get().size() == 0) {
                return false;
            } else {
                return true;
            }
        }
    };

    public SmartCam cameras[] = {
            new SmartCam("beedril"),
            new SmartCam("vespiquen"),
            new SmartCam("combee")
    };

    private PhotonPoseEstimator estimatePoseFromCamera(SmartCam Camera) {
        Transform3d relativeCameraPosition = null;
        switch (Camera.name) {

            case "combee": // 270
                relativeCameraPosition = new Transform3d(new Translation3d(0.1016, -0.32385, 0.4238625),
                        new Rotation3d(0, 0, Math.toRadians(270)));
                break;

            case "beedril": // 180
                relativeCameraPosition = new Transform3d(new Translation3d(-0.263525, 0.00635, 0.41275),
                        new Rotation3d(0, 0, Math.toRadians(180)));
                break;

            case "vespiquen": // 90
                relativeCameraPosition = new Transform3d(new Translation3d(-0.2794, 0.2667, 0.29845),
                        new Rotation3d(0, 0, Math.toRadians(90)));
                break;

            default:
                System.err.println("bad camera name");
                break;
        }
        return new PhotonPoseEstimator(kTagFieldLayout, relativeCameraPosition);
    }

    public void periodic() {
        // this is in a seperate function so it can be quickly disabled if needed
        VisionLoop();
    }

    private void VisionLoop() {
        for (SmartCam c : cameras) {
            c.update();
        }
        prettySmartDashboardPose(compiledRobotPose(), "all", false);

        SmartDashboard.putNumber("failed_samples", failed_samples);
        SmartDashboard.putNumber("multitag_samples", multitag_samples);
        SmartDashboard.putNumber("basic_samples", basic_samples);

    }

    public List<PhotonTrackedTarget> getAllTargets() {
        List<PhotonTrackedTarget> compiledTargets = new ArrayList<>();

        for (SmartCam c : cameras) {
            if (!c.SafeResSafe()) {
                continue;
            }
            for (PhotonPipelineResult cR : c.SafeResults().get()) {
                compiledTargets.addAll(cR.getTargets());
            }
        }
        return compiledTargets;
    }

    public List<PhotonTrackedTarget> getCameraTargets(String cameraName) {
        List<PhotonTrackedTarget> compiledResults = new ArrayList<>();
        for (SmartCam c : cameras) {
            if (c.name == cameraName) {
                if (!c.SafeResSafe()) {
                    continue;
                }
                for (PhotonPipelineResult sR : c.SafeResults().get()) {
                    compiledResults.addAll(sR.getTargets());
                }
            }
        }

        return compiledResults;
    }

    public void prettySmartDashboardPose(Optional<Pose3d> inputPose, String logMode, boolean outputInches) {
        // log modes:
        // "all" - logs x,y,z,rot,field
        // "posbasic" - logs x,y,z
        // "posfull" - logs x,y,z,rot
        // "field" - just field

        // default output is in meters

        if (inputPose.isPresent()) {
            Pose3d goodPose = inputPose.get();
            switch (logMode) {
                case "field":
                    vision_robot_pose.setRobotPose(goodPose.getX(), goodPose.getY(),
                            goodPose.getRotation().toRotation2d());
                    SmartDashboard.putData("vision_est_field", vision_robot_pose);
                    break;
                case "posbasic":
                    if (outputInches) {
                        SmartDashboard.putNumber("vposx", goodPose.getX() * 39.37008);
                        SmartDashboard.putNumber("vposy", goodPose.getY() * 39.37008);
                        SmartDashboard.putNumber("vposz", goodPose.getZ() * 39.37008);
                    } else {
                        SmartDashboard.putNumber("vposx", goodPose.getX());
                        SmartDashboard.putNumber("vposy", goodPose.getY());
                        SmartDashboard.putNumber("vposz", goodPose.getZ());
                    }
                    break;
                case "posfull":
                    SmartDashboard.putString("vposrot", goodPose.getRotation().toString());
                    prettySmartDashboardPose(inputPose, "posbasic", outputInches);
                    break;
                case "all":
                    prettySmartDashboardPose(inputPose, "posfull", outputInches);
                    prettySmartDashboardPose(inputPose, "field", outputInches);
                    break;
                default:
                    break;
            }
        }

    }

    public Optional<Pose3d> compiledRobotPose() {
        // tested and functional !!!!!!!!!
        // uses several cameras to result in one position
        // average positions to reduce miss
        List<EstimatedRobotPose> estimatedPositions = new ArrayList<>();
        for (SmartCam c : cameras) {

            if (!c.SafeResSafe()) {
                continue;
            }

            for (PhotonPipelineResult item : c.SafeResults().get()) {
                Optional<EstimatedRobotPose> constructorObject = estimatePoseFromCamera(c)
                        .estimateCoprocMultiTagPose(item);

                if (constructorObject.isPresent()) {
                    multitag_samples++;
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }

                // backup estimation method
                constructorObject = estimatePoseFromCamera(c)
                        .estimateLowestAmbiguityPose(item);

                if (constructorObject.isPresent()) {
                    basic_samples++;
                    estimatedPositions.add(constructorObject.get());
                    continue;
                }
                // System.err.println("Both pose estimation types failed to find any tags.");
                failed_samples++;
            }
        }

        if (estimatedPositions.size() < MIN_POS_SAMPLE) {

            return Optional.empty();
        }

        // return mean of all estimations
        Translation3d avgTranslation = new Translation3d(); // seperate into translation and rotation
        Rotation3d avgRotation = new Rotation3d();
        for (EstimatedRobotPose estPos : estimatedPositions) {
            avgTranslation = avgTranslation.plus(estPos.estimatedPose.getTranslation()); // add all together
                                                                                         // seperately
            avgRotation = avgRotation.plus(estPos.estimatedPose.getRotation());
        }

        Pose3d completePose = new Pose3d(avgTranslation, avgRotation); // recombine
        completePose = completePose.div(estimatedPositions.size()); // divide by len

        return roundPose3d(Optional.of(completePose));
    }

    public Optional<Pose3d> roundPose3d(Optional<Pose3d> inp) {
        if (inp.isEmpty()) {
            return Optional.empty();
        }
        Pose3d constructorPose = inp.get();

        // position
        double[] positionArray = {
                constructorPose.getX(),
                constructorPose.getY(),
                constructorPose.getZ()
        };
        for (int i = 0; i < positionArray.length; i++) {
            positionArray[i] = (Math.round(positionArray[i] * Math.pow(10, DECI_ACC))) / Math.pow(10, DECI_ACC);
        }
        // rounding rotation really isnt worth the performance hit

        inp = Optional.of(new Pose3d(positionArray[0], positionArray[1], positionArray[2], inp.get().getRotation()));

        return inp;
    }

    public Optional<Pose3d> turretPose() {
        // just completePose modified to be offset from turret point

        final Transform3d turretOffset = new Transform3d();
        final Optional<Pose3d> crp = compiledRobotPose();
        if (crp.isPresent()) {
            return Optional.of(compiledRobotPose().get().plus(turretOffset));
        } else {
            return Optional.empty();
        }

    }

    public Pose3d tagPose(PhotonTrackedTarget AprilTag) {
        // untested?
        return new Pose3d(
                new Translation3d(AprilTag.bestCameraToTarget.getX(), AprilTag.bestCameraToTarget.getY(),
                        AprilTag.bestCameraToTarget.getZ()),
                new Rotation3d(AprilTag.skew, AprilTag.pitch, AprilTag.yaw));
    }

    public Optional<Rotation2d> rotationToTag(int fidID, Boolean fromTurret) {
        // untested
        // fiducial ID of april tag
        // and True if centered on turret,
        // False if from the center of the robot
        // returns a transform from the center to the target tag

        PhotonTrackedTarget locatedFID = null;
        Pose3d tagPose3d;
        Optional<Pose3d> selfPose3d;
        if (fromTurret) {
            selfPose3d = turretPose();
        } else {
            selfPose3d = compiledRobotPose();
        }
        if (selfPose3d.isEmpty()) {
            return Optional.empty();
        }
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

        return Optional.of(PhotonUtils.getYawToPose(selfPose3d.get().toPose2d(), tagPose3d.toPose2d()));
    }
}
