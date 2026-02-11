package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Universals;

public class ClimbSubsystem extends SubsystemBase{

    private final SparkMax climber = new SparkMax(7, MotorType.kBrushless);

    RelativeEncoder climbEncoder;
    public double climbEncoderPos;
    
    private SparkMaxConfig climbConfig = new SparkMaxConfig();
    private SparkClosedLoopController climbClosedLoopController;

    public double targetPosition = 0;


    public ClimbSubsystem(){

        initPid();         

    }

    private void initPid(){
        // do your pid initialization here
        climbClosedLoopController = climber.getClosedLoopController();
        climbEncoder = climber.getEncoder();

        
        climbConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                climber.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
    }

    public void periodic(){

        climbEncoderPos = climbEncoder.getPosition();

        climbClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("ClimbEncoderPos", climbEncoderPos);
        SmartDashboard.putNumber("ClimbTarget", targetPosition);

        // do your pid calculation here (use targetPosition!)
    }

    public void reBoot(){
        climbEncoder.setPosition(0);
    }
}
