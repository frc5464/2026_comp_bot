package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

public class Vision extends SubsystemBase {

    public PhotonCamera[] camera = {
            new PhotonCamera("driver")
    };

    public List<PhotonPipelineResult> newest_i(String targ) {
        // for each camera, check if it matches target and return results if so
        for (int i = 0; i < camera.length; i++) {
            if (camera[i].getName() == targ) {
                return camera[i].getAllUnreadResults();
            }

        }
        System.out.println("Camera not found. This is bad !! check for typos");
        return null;
    }

    public List<PhotonTrackedTarget> targets_i(String targ) {
        // source pipeline results
        List<PhotonPipelineResult> origin = newest_i(targ);

        // result compiled list
        List<PhotonTrackedTarget> RCL = new ArrayList<>();

        boolean has_target = false;
        for (int i = 0; i < origin.size(); i++) {
            if (origin.get(i).hasTargets()) {
                has_target = true;
                break;
            }
        }
        if (!has_target) {
            System.out.println("Not targetful, exiting iteration");
            return null;
        }
        for (int i = 0; i < origin.size(); i++) {
            if (origin.get(i).hasTargets()) {
                RCL.addAll(origin.get(i).getTargets());
            }
        }
        if (RCL.isEmpty()) {
            System.out.println("warning: empty outs");
        }
        return RCL;

    }

}