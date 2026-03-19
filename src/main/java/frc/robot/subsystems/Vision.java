package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public PhotonCamera[] camera = {
            new PhotonCamera("driver") /*driver cam wont actually be used for vision due to variable position */
    };

    // @Override
    public void periodic(){
        //this is bad and should be removed most likely
        //the List<PhotonPipelineResult> isnt used for anything
        //and this doesnt need to be run periodicly
        //right now at least
        //eventually this might be a good idea if it's referenced by more then one other function at a given moment
        newest_scan("allVision");
    }

    public List<PhotonPipelineResult> newest_scan(String targ) {
        // for each camera, check if it matches target and return results if so
        List<PhotonPipelineResult> compiled_targets = new ArrayList<>();
        for (int i = 0; i < camera.length; i++) {
            if (camera[i].getName() == targ) {
                SmartDashboard.putString("AT found","OC/"+camera[i].getAllUnreadResults().toString());
                return camera[i].getAllUnreadResults();
            }
            if (targ == "allVision" && camera[i].getName() != "driver"){
                compiled_targets.addAll(camera[i].getAllUnreadResults());
            }
            SmartDashboard.putString("AT found","AV/"+camera[i].getAllUnreadResults().toString());
        
        
            
        }


        if (targ == "allVision"){
            // used if you want all results instead of just those of a specfic camera
            return compiled_targets;
        }
        System.out.println("Camera not found. This is bad !! check for typos");
        return null;
    }

    public List<PhotonTrackedTarget> targets_all(String targ) {
        // source pipeline results
        List<PhotonPipelineResult> origin = newest_scan(targ);
        
        // result compiled list
        List<PhotonTrackedTarget> RCL = new ArrayList<>();
        SmartDashboard.putString("AT targets",RCL.toString());
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

    public void rotate_to_tag(int targetAprilTag){
        List<PhotonTrackedTarget> all_detected = targets_all("allVision"); /*pass empty to use all cameras */
        
        for (PhotonTrackedTarget at : all_detected) {
            if (at.getFiducialId() == targetAprilTag){
                SmartDashboard.putBoolean("winner", true);
                //TODO: implement rotation
                //  take corners of given at + area + pitch/yaw/rotation
                //  rotate shooter (after reset) until match?
                //  check in with gabe/michael on how the shooter works
                //  actually get to test the code ???
            }
        }
    }
}