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
    // RelativeEncoder rightEncoder;
    public double encoderPos;
    // public double rightEncoderPos;
    public double counts;
    public int jawPosition = 0;
    PIDController jawPID;

    public double targetPosition = 0.5;

    // private SparkMaxConfig leftRotConfig = new SparkMaxConfig();
    // private SparkMaxConfig rightRotConfig = new SparkMaxConfig();
    // private SparkClosedLoopController leftController = leftRotator.getClosedLoopController();
    // private SparkClosedLoopController rightController = rightRotator.getClosedLoopController();
    // private RelativeEncoder leftrelativeEncoder;
    // private RelativeEncoder rightRelativeEncoder;
    // private double maxPower = 0.3;

    public IntakeSubsystem(){
        jawPID = new PIDController(kP, kI, kD);

        leftEncoder = leftJaw.getEncoder();
        
        SparkBaseConfig conf = new SparkMaxConfig();
        conf.idleMode(IdleMode.kBrake);
        conf.openLoopRampRate(0.5);

        leftJaw.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig conf2 = new SparkMaxConfig();
        conf2.follow(3, true);
        conf2.idleMode(IdleMode.kBrake);
        rightJaw.configure(conf2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
   
   
   
        //     leftController = leftRotator.getClosedLoopController();
    //     rightController = rightRotator.getClosedLoopController();

    //     leftrelativeEncoder = leftRotator.getEncoder();
    //     rightRelativeEncoder = rightRotator.getEncoder();

    //     leftRotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.3).i(0).d(0).outputRange(-0.3, 0.3);
    //     rightRotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.3).i(0).d(0).outputRange(-0.3, 0.3);

    //     leftRotator.configure(leftRotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //     rightRotator.configure(rightRotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void checkForPidChanges(){
    double newP = SmartDashboard.getNumber("elP", -1);
    double newI = SmartDashboard.getNumber("elI", -1);
    double newD = SmartDashboard.getNumber("elD", -1);
    // System.out.println(newP);
    if(newP != kP){
      kP = newP;
      jawPID.setP(newP);
      System.out.println("New P parameter!");
    }
    if(newI != kI){
      kI = newI;
      jawPID.setI(newI);
      System.out.println("New I parameter!");
    }
    if(newD != kD){
      kD = newD;
      jawPID.setD(newD);
      System.out.println("New D parameter!");
    }
  }

    public void periodic(){
        encoderPos = leftEncoder.getPosition();

        SmartDashboard.putNumber("JawEncoder", encoderPos);
        SmartDashboard.putNumber("JawTarget", targetPosition);

        checkForPidChanges();


        // SmartDashboard.putNumber("leftRaiseIntakeEncoder", leftrelativeEncoder.getPosition());
        // SmartDashboard.putNumber("rightRaiseIntakeEncoder", rightRelativeEncoder.getPosition());
        // leftController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // rightController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // SmartDashboard.putBoolean("feeding", Universals.feeding);
        // SmartDashboard.putBoolean("Intaking", Universals.intaking);
    }

    public void jawPIDToLevel(){
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

    // public double getLeftCurrent(){
    //     return leftRotator.getOutputCurrent();
    // }

    // public double getRightCurrent(){
    //     return rightRotator.getOutputCurrent();
    // }

    // public void fuelPickup(){
    //     targetPosition = 1;
    // }

    // public void start(){
    //     targetPosition = 0;
    // }

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
