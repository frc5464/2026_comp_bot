package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    private final SparkMax turret = new SparkMax(53, MotorType.kBrushless);
    private final SparkMax shooter = new SparkMax(54, MotorType.kBrushless);
}
