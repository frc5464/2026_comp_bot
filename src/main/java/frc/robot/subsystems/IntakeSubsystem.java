package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax leftJaw = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax rightJaw = new SparkMax(3, MotorType.kBrushless);
    
    private final SparkMax intakeRod = new SparkMax(8, MotorType.kBrushless);

    RelativeEncoder leftEncoder;
    public double encoderPos;
    
    private SparkMaxConfig motorConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController;

    public double targetPosition = 0;


    public IntakeSubsystem(){

        initPid();         

        SparkBaseConfig conf2 = new SparkMaxConfig();
        conf2.follow(6, false);
        conf2.idleMode(IdleMode.kBrake);
        rightJaw.configure(conf2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    }

    private void initPid(){
        // do your pid initialization here
        closedLoopController = leftJaw.getClosedLoopController();
        leftEncoder = leftJaw.getEncoder();

        
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                leftJaw.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
    }

    public void periodic(){

        encoderPos = leftEncoder.getPosition();

        closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("JawEncoder", encoderPos);
        SmartDashboard.putNumber("JawTarget", targetPosition);

        // do your pid calculation here (use targetPosition!)
    }

    public void reBoot(){
        leftEncoder.setPosition(0);
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
                intakeRod.set(-0.5);
            }


    public void IntakeReverse(){
            intakeRod.set(1);
      
    }

    public void DisableIntake(){
        intakeRod.set(0);
    }
}
