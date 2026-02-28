package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Universals;
import frc.robot.Commands.IntakeToPositionCommand;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax leftJaw = new SparkMax(10, MotorType.kBrushless);
    private final SparkMax rightJaw = new SparkMax(11, MotorType.kBrushless);
    
    // private final SparkMax intakeRod = new SparkMax(12, MotorType.kBrushless);
    private final TalonFX intakeRod = new TalonFX(12);

    RelativeEncoder leftjawEncoder;
    public double leftjawEncoderPos;
    
    private SparkMaxConfig leftjawMotorConfig = new SparkMaxConfig();
    public SparkClosedLoopController leftjawClosedLoopController;

    public double lefttargetPositionInt = -19;


    public IntakeSubsystem(){

        // leftjawEncoder = leftJaw.getEncoder();
        initPid();       
        
        // SparkBaseConfig conf = new SparkMaxConfig();
        // conf.idleMode(IdleMode.kCoast);
        // // conf.openLoopRampRate(0.5);
        // leftJaw.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // follow the yellow brick road (left jaw)
        SparkBaseConfig conf2 = new SparkMaxConfig();
        conf2.follow(10, false);
        conf2.idleMode(IdleMode.kBrake);
        rightJaw.configure(conf2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    }

    private void initPid(){
        // do your pid initialization here
        leftjawClosedLoopController = leftJaw.getClosedLoopController();
        leftjawEncoder = leftJaw.getEncoder();

        
        leftjawMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-0.2, 0.2)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                leftJaw.configure(leftjawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
    }

    public void periodic(){
        if(Universals.manualMode == false){
            leftjawEncoderPos = leftjawEncoder.getPosition();
            leftjawClosedLoopController.setSetpoint(lefttargetPositionInt, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        SmartDashboard.putNumber("JawEncoder", leftjawEncoderPos);
        SmartDashboard.putNumber("JawTarget", lefttargetPositionInt);

        // do your pid calculation here (use targetPosition!)
    }

    // public void intakePID(){
    //     if(){
    //     }    lefttargetPositionInt = -0.1;
        
    //     if(){
    //         lefttargetPositionInt = -4.8;
    //     }
    //     else{
    //         leftJaw.set(0);
    //     }
    // }

    public void reBoot(){
        leftjawEncoder.setPosition(0);
    }

    public void ManualRaiseIntake(){
        leftJaw.set(0.1);
        // rightJaw.set(0.20);
    }

    public void ManualLowerIntake(){
        leftJaw.set(-0.1);
        // rightJaw.set(-0.10);
    }

    public void intakeUp(){

    }

    public void stopElevate(){
        leftJaw.set(0);
        rightJaw.set(0);
    }

    public void Intake(){
        intakeRod.set(1);
    }


    public void IntakeReverse(){
            intakeRod.set(-1);
      
    }

    public void DisableIntake(){
        intakeRod.set(0);
    }
}
