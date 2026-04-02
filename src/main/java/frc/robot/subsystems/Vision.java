package frc.robot.subsystems;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.linkedCamera;

public class Vision extends SubsystemBase {

    // the apriltag field
    public static final AprilTagFieldLayout kTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    public class linkedCamera {
        PhotonCamera photonCam;
        List<PhotonPipelineResult> lastResults = new ArrayList<>();
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        int ESTID;

        public linkedCamera(PhotonCamera camImport, int estimatorIndex) {
            this.photonCam = camImport;
            // max fps for 20ms periodic
            if (this.photonCam.getFPSLimit() > 50) {
                this.photonCam.setFPSLimit(50);
                this.ESTID = estimatorIndex;
            }
        }

        public void update() {
            this.lastResults = this.photonCam.getAllUnreadResults();
            if (this.lastResults.size() == 0) {
                return;
            }
            for (PhotonPipelineResult lR : this.lastResults) {
                if (!lR.hasTargets()) {
                    continue;
                }
                this.targets = lR.getTargets();

            }
        }

    }
    // index to ESTIMATORS to allow for static estimators but dynamic transformation
    linkedCamera[] cameras = {
            new linkedCamera(new PhotonCamera("ZSC"), 0),
            new linkedCamera(new PhotonCamera("LPC"), 1)
    };
    static final PhotonPoseEstimator[] ESTIMATORS = {
            new PhotonPoseEstimator(kTagFieldLayout, Transform3d.kZero),
            new PhotonPoseEstimator(kTagFieldLayout, Transform3d.kZero)
    };

    // define every variable now
    Field2d visionField = new Field2d();
    Optional<EstimatedRobotPose> constructorPose = Optional.empty();
    Translation3d constructorTranslation = new Translation3d();
    Rotation3d constructorRotation = new Rotation3d();
    List<EstimatedRobotPose> calcPose = new ArrayList<>();
    int poseCount = 0;

    public void periodic() {
        // visionPeriodic();
    }

    public void visionPeriodic() {
        // clear pose before starting
        // doesn't reset to 0,0,0 due to checks later
        calcPose.clear();
        for (linkedCamera c : cameras) {
            c.update();
            if (c.lastResults.size() == 0) {
                continue;
            }
            constructorPose = ESTIMATORS[c.ESTID].estimateLowestAmbiguityPose(c.lastResults.get(0));
            if (constructorPose.isEmpty()) {
                continue;
            }
            calcPose.add(constructorPose.get());
        }
        // clear variables before new iteration
        constructorRotation = new Rotation3d();
        constructorTranslation = new Translation3d();
        poseCount = 0;

        // mean (again)
        for (EstimatedRobotPose pose : calcPose) {
            constructorRotation = constructorRotation.plus(pose.estimatedPose.getRotation());
            constructorTranslation = constructorTranslation.plus(pose.estimatedPose.getTranslation());
            poseCount++;
        }
        // poseCount checked to prevent 0,0,0
        if (poseCount != 0) {
            constructorRotation = constructorRotation.div(poseCount);
            constructorTranslation = constructorTranslation.div(poseCount);
            visionField.setRobotPose(new Pose3d(constructorTranslation, constructorRotation).toPose2d());
            SmartDashboard.putData("visionField", visionField);
        }

    }
}
