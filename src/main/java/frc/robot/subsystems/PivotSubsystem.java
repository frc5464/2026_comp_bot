package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class PivotSubsystem {

    public SparkMax pivot1 = new SparkMax(11, MotorType.kBrushless);
    public SparkMax pivot2 = new SparkMax(12, MotorType.kBrushless);

    public void pivot(){
        pivot1.set(1);
        pivot2.set(-1);
    }
    

    
}
