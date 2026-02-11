package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase{
    
    private final SparkMax belt = new SparkMax(57, MotorType.kBrushless);
    
    public void runBelt(){
        belt.set(1);
    }

    public void stopBelt(){
        belt.set(0);
    }
}
