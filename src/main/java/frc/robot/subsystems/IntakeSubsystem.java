package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    public SparkMax intake = new SparkMax(10, MotorType.kBrushless);
   

    public void intake(){
        intake.set(1);
    }

    public void outake(){
        intake.set(-1);
    }

    public void stop(){
        intake.set(0);
    }

}
