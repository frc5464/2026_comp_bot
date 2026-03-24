package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    
    private class lookupTable {
        // functions as a dictionary,
        // where the index of a key is used as the index of a result
        // interact with using functions, not directly
        // key/result mismatch may otherwise occur

        List<String> keys = new ArrayList<>();
        List<List<PhotonPipelineResult>> results = new ArrayList<>();

        public int addItem(String Key, List<PhotonPipelineResult> value){
            
            if (!keys.contains(Key)){
                keys.add(Key);
                results.add(value);
            } else {
                results.set(keys.indexOf(Key),value);
            }
            
            return keys.indexOf(Key);
        }

        public List<PhotonPipelineResult> retrieveItem(String Key){
            return results.get(keys.indexOf(Key));
        }

        public void popItem(String Key){
            results.remove(keys.indexOf(Key));
            keys.remove(Key);
        }
        public void clear(){
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

    private Transform3d turretOffset = new Transform3d();

    public void periodic(){
        // dont uncomment and push to main till stable
        //VisionLoop();
    }




    public void VisionLoop(){

        cameraTable.clear(); /* clear table before adding items again */
        
        for (PhotonCamera c : cameras) {

            // set the relative turret offset for a given camera
            // default value also included
            // TODO: replace with actual offsets.
            switch (c.getName()) {
                case "ribombee":
                    turretOffset = new Transform3d();
                    break;
                
                case "combee":
                    turretOffset = new Transform3d();
                    break;

                case "beedril":
                    turretOffset = new Transform3d();
                    break;
                
                case "vespiquen":
                    turretOffset = new Transform3d();
                    break;
                
                default:
                    turretOffset = new Transform3d();
                    break;
            }
            cameraTable.addItem(c.getName(),c.getAllUnreadResults());
        }

        for (String key : cameraTable.keys) {
            SmartDashboard.putString(key,cameraTable.retrieveItem(key).toString());
        }
    }
}