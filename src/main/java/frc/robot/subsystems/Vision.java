package frc.robot.subsystems;


import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

public class Vision extends SubsystemBase {



    public PhotonCamera[] camera = {
        new PhotonCamera("driver")
    };

    

    
    public List<PhotonPipelineResult> newest_i(String targ){

        for (int i = 0; i < camera.length; i++) {
            if (camera[i].getName() == targ) {
                return camera[i].getAllUnreadResults();
            }
               
        }
        System.out.println("Camera not found. This is bad !! check for typos");
        return null;
    }

}