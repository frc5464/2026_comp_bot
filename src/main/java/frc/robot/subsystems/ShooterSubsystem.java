package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{

    private final SparkMax shooterMotor = new SparkMax(4, MotorType.kBrushless/*ShooterConstants.kShooterMotorPort*/);
    private final SparkMax feederMotor = new SparkMax(5, MotorType.kBrushless/*ShooterConstants.kFeederMotorPort*/);
    private final SparkMax shootHinge = new SparkMax(76, MotorType.kBrushless);

    //Stuff for shootPosition PID
    public RelativeEncoder hingeEncoder;
    public double encoderPos;
    
    private SparkMaxConfig posConfig = new SparkMaxConfig();
    private SparkClosedLoopController posClosedLoopController;

    public double targetPosition = 0;

    //Stuff for shootVelocity PID
    RelativeEncoder flyEncoder;
    public double encoderVel;

    private SparkMaxConfig flyConfig = new SparkMaxConfig();
    private SparkClosedLoopController flyClosedLoopController;

    public double targetVelocity = 0;

  public ShooterSubsystem(){

      initPidShoot();

  }


    
  
  private void initPidShoot(){

      // Position PID for shoot hinge
        // do your pid initialization here
        posClosedLoopController = shootHinge.getClosedLoopController();
        hingeEncoder = shootHinge.getEncoder();

        
        posConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                shootHinge.configure(posConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);

      // Velocity PID for shooter
        flyClosedLoopController = shooterMotor.getClosedLoopController();
        flyEncoder = shooterMotor.getEncoder();

        flyConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

                shooterMotor.configure(flyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      SmartDashboard.setDefaultNumber("ShootTargetVel", 0);
  }

  public void periodic(){
      // Shooter Code
      encoderVel = flyEncoder.getVelocity();

      flyClosedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      
      SmartDashboard.putNumber("ShootVelocity", encoderVel);
      SmartDashboard.putNumber("FlyEncoder", targetVelocity);

      // ShootRot Code
      encoderPos = hingeEncoder.getPosition();

      posClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

      SmartDashboard.putNumber("JawEncoder", encoderPos);
      SmartDashboard.putNumber("JawTarget", targetPosition);
    }
  
  public void feed(){
    feederMotor.set(0.75);
  }

  public void reverseShoot(){
    shooterMotor.set(-1);
  }

  public void disableShoot(){
    shooterMotor.set(0);
    feederMotor.set(0);
  }

  public void reBoot(){
      hingeEncoder.setPosition(0);
  }
}