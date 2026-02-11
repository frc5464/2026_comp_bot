package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase{
    
    private final SparkMax turret = new SparkMax(7, MotorType.kBrushless);

    public RelativeEncoder turretEncoder;
    public double turretEncoderPos;
    
    private SparkMaxConfig climbConfig = new SparkMaxConfig();
    private SparkClosedLoopController climbClosedLoopController;

    public double targetPosition = 0;


    public TurretSubsystem(){

        initPidTurret();         

    }

    private void initPidTurret(){
        // do your pid initialization here
        climbClosedLoopController = turret.getClosedLoopController();
        turretEncoder = turret.getEncoder();

        
        climbConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                turret.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
    }

    public void periodic(){

        turretEncoderPos = turretEncoder.getPosition();

        climbClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("turretEncoderPos", turretEncoderPos);
        SmartDashboard.putNumber("TurretTarget", targetPosition);

        // do your pid calculation here (use targetPosition!)
    }

    public void reBoot(){
        turretEncoder.setPosition(0);
    }

}
