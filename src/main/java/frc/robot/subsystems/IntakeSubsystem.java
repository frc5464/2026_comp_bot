package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Universals;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax leftRotator = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax rightRotator = new SparkMax(51, MotorType.kBrushless);
    
    private final SparkMax intakeRod = new SparkMax(52, MotorType.kBrushless);

    SparkMaxConfig sparkMaxConfig1;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double extMaxOutput = 0;
    double extMinOutput = 0;
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    public double leftEncoderPos;
    public double rightEncoderPos;
    public double counts;
    public int level = 0;

    public double targetPosition = 0.5;

    private SparkMaxConfig leftRotConfig = new SparkMaxConfig();
    private SparkMaxConfig rightRotConfig = new SparkMaxConfig();
    private SparkClosedLoopController leftController = leftRotator.getClosedLoopController();
    private SparkClosedLoopController rightController = rightRotator.getClosedLoopController();
    private RelativeEncoder leftrelativeEncoder;
    private RelativeEncoder rightRelativeEncoder;
    private double maxPower = 0.3;

    public IntakeSubsystem(){
        leftController = leftRotator.getClosedLoopController();
        rightController = rightRotator.getClosedLoopController();

        leftrelativeEncoder = leftRotator.getEncoder();
        rightRelativeEncoder = rightRotator.getEncoder();

        leftRotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.3).i(0).d(0).outputRange(-0.3, 0.3);

        leftRotator.configure(leftRotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void periodic(){
        SmartDashboard.putNumber("leftIntakeEncoder", leftrelativeEncoder.getPosition());
        leftController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // SmartDashboard.putBoolean("feeding", Universals.feeding);
        // SmartDashboard.putBoolean("Intaking", Universals.intaking);
    }

    public void ManualRaiseIntake(){
        leftRotator.set(1);
        // rightRotator.set(-1);
    }

    public void ManualLowerIntake(){
        leftRotator.set(-1);
        // rightRotator.set(1);
    }

    public void intakeUp(){

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
}
