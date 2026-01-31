package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Universals;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax leftRotator = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax rightRotator = new SparkMax(51, MotorType.kBrushless);
    
    private final SparkMax intakeRod = new SparkMax(52, MotorType.kBrushless);

    // private final Encoder encoder = new Encoder(null, null);

    // private final PIDController intakeFeedback = new PIDController(0, 0, 0);
    // DigitalInput searchFuel = new DigitalInput(0);
    // private static final boolean ENABLED = true;
    // public boolean fuelnotdetected = false;

    public void periodic(){
        // SmartDashboard.putBoolean("feeding", Universals.feeding);
        SmartDashboard.putBoolean("Intaking", Universals.intaking);
    }

    public void ManualRaiseIntake(){
        leftRotator.set(1);
        rightRotator.set(-1);
    }

    public void ManualLowerIntake(){
        leftRotator.set(-1);
        rightRotator.set(1);
    }

    public void stopElevate(){
        leftRotator.set(0);
        rightRotator.set(0);
    }

    public void Intake(){
        // if(Universals.intaking){
            // if(fuelnotdetected == true){
                intakeRod.set(-1);
            // }
            // else{
                // intakeRod.set(0);
                // Universals.LEDselected = "PinkIntake";
            }
        // }
        // else{
            // DisableIntake();
        // }
    // }

    public void IntakeReverse(){
        // if(Universals.intakeReverse){
            intakeRod.set(1);
        // }
        // else{
        //     DisableIntake();
        // }
    }

    public void DisableIntake(){
        intakeRod.set(0);
    }

    // public void IntakeFeed(){
        // if (Universals.feeding) {
            // intakeRod.set(-1);
        // }
        // else{
            // DisableIntake();
            // Universals.LEDselected = "White";
        // }
        // }
}
