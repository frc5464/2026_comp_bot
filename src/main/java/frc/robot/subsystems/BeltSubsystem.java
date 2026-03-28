package frc.robot.subsystems;

// import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase{
    
    private final SparkMax belt = new SparkMax(13, MotorType.kBrushless);
    private final SparkMax agitator = new SparkMax(19, MotorType.kBrushless);
    
    public void periodicrunbelt(){
        // runBelt();
    }
    
    public void runBelt(double beltspd){
        belt.set(beltspd);
        agitator.set(beltspd);
    }

    public void stopBelt(){
        belt.set(0);
        agitator.set(0);
    }
}
