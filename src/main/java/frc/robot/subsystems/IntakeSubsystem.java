package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Universals;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax leftJaw = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax rightJaw = new SparkMax(3, MotorType.kBrushless);
    
    private final SparkMax intakeRod = new SparkMax(8, MotorType.kBrushless);

    SparkMaxConfig sparkMaxConfig1 = new SparkMaxConfig();
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double extMaxOutput = 0;
    double extMinOutput = 0;
    RelativeEncoder leftEncoder;
    public double encoderPos;
    public double counts;
    public int jawPosition = 0;
    PIDController jawPID;

    public double targetPosition = 0.5;


    public IntakeSubsystem(){
        jawPID = new PIDController(kP, kI, kD);

        initPid(); 

        leftEncoder = leftJaw.getEncoder();
        
        SparkBaseConfig conf = new SparkMaxConfig();
        conf.idleMode(IdleMode.kBrake);
        conf.openLoopRampRate(0.5);

        leftJaw.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig conf2 = new SparkMaxConfig();
        conf2.follow(3, true);
        conf2.idleMode(IdleMode.kBrake);
        rightJaw.configure(conf2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    }

    private void initPid(){
        // do your pid initialization here
    }

    public void periodic(){
        encoderPos = leftEncoder.getPosition();

        SmartDashboard.putNumber("JawEncoder", encoderPos);
        SmartDashboard.putNumber("JawTarget", targetPosition);

        // do your pid calculation here (use targetPosition!)
    }

 
    public void jawPIDToLevel(){
        // (make this modify targetPosition)
        if(jawPosition == 1){
            encoderPos = -4.5;
        }
        if(jawPosition == 2){
            encoderPos = 0.2;
        }
        else{
            leftJaw.set(0);
        }
    }

    public void reBoot(){
        leftEncoder.setPosition(0);
    }

    public void ManualRaiseIntake(){
        leftJaw.set(0.20);
        rightJaw.set(0.20);
    }

    public void ManualLowerIntake(){
        leftJaw.set(-0.10);
        rightJaw.set(-0.10);
    }

    public void intakeUp(){

    }

    public void stopElevate(){
        leftJaw.set(0);
        rightJaw.set(0);
    }

    public void Intake(){
        // if(Universals.intaking){
            // if(fuelnotdetected == true){
                intakeRod.set(-0.5);
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
}
