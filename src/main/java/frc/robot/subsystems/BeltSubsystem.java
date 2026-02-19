package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase{
    
    private final SparkFlex belt = new SparkFlex(13, MotorType.kBrushless);
    
    public void periodic(){
        runBelt();
    }
    
    public void runBelt(){
        belt.set(1);
    }

    public void stopBelt(){
        belt.set(0);
    }
}
