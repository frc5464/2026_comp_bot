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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase{

    // private final SparkMax turret = new SparkMax(14, MotorType.kBrushless);

    // public RelativeEncoder turretEncoder;
    // public double turretEncoderPos;
    public double angle;
    
    // private SparkMaxConfig turretConfig = new SparkMaxConfig();
    // private SparkClosedLoopController turretClosedLoopController;

    // public double targetPosition = 0;


    // public TurretSubsystem(){

    //     initPidTurret();         

    // }

    // private void initPidTurret(){
    //     // do your pid initialization here
    //     turretClosedLoopController = turret.getClosedLoopController();
    //     turretEncoder = turret.getEncoder();

        
    //     turretConfig.closedLoop
    //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //         //Set PID values for position control
    //         .p(0.1)
    //         .i(0)
    //         .d(0)
    //         .outputRange(-1, 1)
    //         .feedForward
    //             .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

    //             turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //     SmartDashboard.setDefaultNumber("Target Position", 0);
    // }

    // public void periodic(){

    //     turretEncoderPos = turretEncoder.getPosition();

    //     turretClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    //     SmartDashboard.putNumber("turretEncoderPos", turretEncoderPos);
    //     SmartDashboard.putNumber("TurretTarget", targetPosition);

    //     // do your pid calculation here (use targetPosition!)
    // }

    // public void reBoot(){
    //     turretEncoder.setPosition(0);
    // }
    public void autoAim(double x, double y){
        
        // Figure out which hub we need to be aiming at
        if(DriverStation.getAlliance().get() == Alliance.Blue){

            // Calculate the angle needed between turret and blue hub in degrees
            angle =  Math.toDegrees(Math.atan((x-4.6)/(y-4)));
        }

        else{

            // Calculate the angle needed between turret and red hub in degrees
            angle =  Math.toDegrees(Math.atan((x-11.9)/(y-4)));            
        }

        SmartDashboard.putNumber("Turret Angle", angle);
    }
}