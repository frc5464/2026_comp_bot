package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    private final SparkMax turret = new SparkMax(53, MotorType.kBrushless);
    private final SparkMax shooter = new SparkMax(54, MotorType.kBrushless);

    public Command runShooter(){
        return runOnce(
            () -> {
                shooter.set(1);
            });
    }

    public Command turret(){
        return run(
            () -> {
                /* Auto target an april tag attatched to the hub */
            });
    }
}
