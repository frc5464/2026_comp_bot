package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Universals;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax leftRotator = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax rightRotator = new SparkMax(51, MotorType.kBrushless);
    
    private final SparkMax intakeRod = new SparkMax(52, MotorType.kBrushless);

    // DigitalInput searchFuel = new DigitalInput(0);
    // private static final boolean ENABLED = true;
    // public boolean fuelnotdetected = false;

    public void periodic(){
        //grab the past state of the break beam
        // boolean oldfuelnotdetected = fuelnotdetected;
        //grab the newest state of the break beam
        // fuelnotdetected = !searchFuel.get();
        // Universals.fuelDetected = searchFuel.get();

        //IF something happened (either a new note was detected or a fuel left,)
        // if(fuelnotdetected!=oldfuelnotdetected){
            // if(fuelnotdetected){
                // Universals.LEDselected = "White";
            // }
            // else{
                // Universals.LEDselected = "PinkIntake";
            // }
        // }

        // SmartDashboard.putBoolean("break beam", fuelnotdetected);
        // SmartDashboard.putBoolean("feeding", Universals.feeding);
        // SmartDashboard.putBoolean("Intaking", Universals.intaking);
        // SmartDashboard.putBoolean("Shoot", Universals.shoot);
    }

    // public void RaiseIntake(){
    //     leftRotator.set(1);
    //     rightRotator.set(-1);
    // }

    // public void LowerIntake(){
    //     leftRotator.set(-1);
    //     rightRotator.set(1);
    // }

    // public void stopElevate(){
    //     leftRotator.set(0);
    //     rightRotator.set(0);
    // }

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
