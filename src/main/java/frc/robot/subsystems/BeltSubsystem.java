package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase{
    
    private final SparkMax belt1 = new SparkMax(57, MotorType.kBrushless);
    private final SparkMax belt2 = new SparkMax(58, MotorType.kBrushless); 
}
