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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    // super constructor was buggy, this is static and just works.
    public static final AprilTagFieldLayout kTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    private boolean initialized = false;
    public final static int MIN_POS_SAMPLE = 1;

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
            SmartDashboard.putString(this.name, this.SafeResults().toString());
        }

        public List<PhotonPipelineResult> SafeResults() {
            return results;

        }

        SmartCam cameras[] = {
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
                    SmartDashboard.putBoolean("bad cam", true);
                    break;
            }
            return new PhotonPoseEstimator(kTagFieldLayout, relativeCameraPosition);
        }

        public void init() {
            initialized = true;
        }

        public void periodic() {
            SmartDashboard.putBoolean("AHHHH", initialized);
            if (!initialized) {
                init();
            }

            VisionLoop();
        }

        private void VisionLoop() {
            for (SmartCam c : cameras) {
                SmartDashboard.putString(c.name, "just like anything");
                c.update();
            }
        }

        public List<PhotonTrackedTarget> getAllTargets() {
            List<PhotonTrackedTarget> compiledTargets = new ArrayList<>();

            for (SmartCam c : cameras) {
                if (c.SafeResults().isEmpty()) {
                    continue;
                }
                if (c.SafeResults().size() == 0) {
                    continue;
                }
                for (PhotonPipelineResult cR : c.SafeResults()) {
                    compiledTargets.addAll(cR.getTargets());
                }
            }
            return compiledTargets;
        }

        public List<PhotonTrackedTarget> getCameraTargets(String cameraName) {
            List<PhotonTrackedTarget> compiledResults = new ArrayList<>();
            for (SmartCam c : cameras) {
                if (c.name == cameraName) {
                    if (c.SafeResults().size() == 0) {
                        continue;
                    }
                    for (PhotonPipelineResult sR : c.SafeResults()) {
                        compiledResults.addAll(sR.getTargets());
                    }
                }
            }

            return compiledResults;
        }

        public Optional<Pose3d> compiledRobotPose() {
            // TODO: TEST
            // uses several cameras to result in one position
            // average positions to reduce miss
            SmartDashboard.putString("fishing", "estp");
            List<EstimatedRobotPose> estimatedPositions = new ArrayList<>();
            for (SmartCam c : cameras) {

                if (c.SafeResults().size() == 0) {
                    continue;
                }

                for (PhotonPipelineResult item : c.SafeResults()) {
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
                    // System.err.println("Both pose estimation types failed to find any tags.");
                    SmartDashboard.putBoolean("failedEstimation", true);
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
            return Optional.of(completePose);
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
            return new Pose3d(
                    new Translation3d(AprilTag.bestCameraToTarget.getX(), AprilTag.bestCameraToTarget.getY(),
                            AprilTag.bestCameraToTarget.getZ()),
                    new Rotation3d(AprilTag.skew, AprilTag.pitch, AprilTag.yaw));
        }

        public Optional<Rotation2d> rotationToTag(int fidID, Boolean fromTurret) {
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
}