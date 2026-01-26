package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    
    private final SparkMax leftClimbMotor = new SparkMax(55, MotorType.kBrushless);
    private final SparkMax rightClimbMotor = new SparkMax(56, MotorType.kBrushless);

    public Command extend(){
        return runOnce(
            () -> {
                leftClimbMotor.set(.5);
                rightClimbMotor.set(.5);
            });
    }

    public Command retract(){
        return runOnce(
            () -> {
                leftClimbMotor.set(-.5);
                rightClimbMotor.set(-.5);
            });
    }
}
