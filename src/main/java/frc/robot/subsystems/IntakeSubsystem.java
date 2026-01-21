package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    public IntakeSubsystem(){}

    private final SparkMax leftRotator = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax rightRotator = new SparkMax(51, MotorType.kBrushless);
    private final SparkMax intakeRod = new SparkMax(52, MotorType.kBrushless);

    public Command firstCommand(){
        return runOnce(
            () -> {
                //run whatever here
            });
    }

    public boolean condition(){
        //Check for  boolean condition
        return false;
    }

    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){}
}
