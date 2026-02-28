package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
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
import frc.robot.Universals;
import frc.robot.Commands.ShootCommand;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{

    public final TalonFX shooterMotor = new TalonFX(16/*ShooterConstants.kShooterMotorPort*/);
    public final SparkFlex feederMotor = new SparkFlex(17, MotorType.kBrushless/*ShooterConstants.kFeederMotorPort*/);
    public final SparkMax shootHinge = new SparkMax(18, MotorType.kBrushless);

    //Stuff for shootPosition PID
    public RelativeEncoder hingeEncoder;
    public double encoderPos;
    
    private SparkMaxConfig posConfig = new SparkMaxConfig();
    private SparkClosedLoopController posClosedLoopController;

    public double targetPosition = 0.5;
    public double usualResistance = 1.0;
    //Stuff for shootVelocity PID
    RelativeEncoder flyEncoder;
    public double encoderVel;

    private SparkMaxConfig flyConfig = new SparkMaxConfig();
    public SparkClosedLoopController flyClosedLoopController;

    public double targetVelocity = 0;

    public VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

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
            .outputRange(-0.2, 0.2)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                shootHinge.configure(posConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("ShootHoodTargetPosition", 0);


     //VelocityPID for shooter with Krakens

      // in init function, set slot 0 gains
      var slot0Configs = new Slot0Configs();
      slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0; // no output for integrated error
      slot0Configs.kD = 0; // no output for error derivative

      shooterMotor.getConfigurator().apply(slot0Configs);

      // // create a velocity closed-loop request, voltage output, slot 0 configs
      // final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

      // // set velocity to 8 rps, add 0.5 V to overcome gravity
      // shooterMotor.setControl(m_request.withVelocity(8).withFeedForward(0.5));


      // Velocity PID for shooter with SparkMax
        // flyClosedLoopController = shooterMotor.getClosedLoopController();
        // flyEncoder = shooterMotor.getEncoder();

        // flyConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .p(0.1)
        //     .i(0)
        //     .d(0)
        //     .outputRange(-1, 1)
        //     .feedForward
        //         .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        //         shooterMotor.config(flyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      SmartDashboard.setDefaultNumber("ShootTargetVel", 0);
  }

  public void periodic(){

      // Shooter Code with Krakens
      // create a velocity closed-loop request, voltage output, slot 0 configs
      // if(Universals.shootReverse == false){
        // public VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
          shooterMotor.setControl(m_request.withVelocity(targetVelocity).withFeedForward(0));
        // set velocity to 8 rps, add 0.5 V to overcome gravity
        // shooterMotor.setControl(m_request.withVelocity(targetVelocity).withFeedForward(0.5));

        encoderVel = shooterMotor.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("shootvel", encoderVel);
      // }

      // Shooter Code with SparkMax
      // encoderVel = flyEncoder.getVelocity();

      // flyClosedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      
      // SmartDashboard.putNumber("ShootVelocity", encoderVel);
      // SmartDashboard.putNumber("FlyEncoder", targetVelocity);

      // ShootRot Code
      encoderPos = hingeEncoder.getPosition();

      posClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

      SmartDashboard.putNumber("ShootRotEncoder", encoderPos);
      SmartDashboard.putNumber("ShootRotTarget", targetPosition);
    }
  
  public void feed(){
    feederMotor.set(.75);
  }

  // public void reverseShoot(){
  //   shooterMotor.set(-1);
  // }

  public void disableShoot(){
    shooterMotor.set(0);
    // feederMotor.set(0);
  }

  public void disableFeed(){
    feederMotor.set(0);
  }

  public void changeAngle(double xdistance, double ydistance){
    if(shootHinge.getOutputCurrent() > usualResistance){
      shootHinge.set(0);
    } else {
    /* Make a Linear relationship between the distance value and the ShootHinge 
     * motor encoder. Tune for smaller #'s. Y^2 - Y^1 / X^2 - X^1 = M. 
     * Y^1 = MX^1 + b (Solve for b)
     */
    // y = mx + b
    // m = slope = (y2 - y1)/(x2 - x1)
      targetPosition = (-40*(Math.hypot(xdistance - 4, ydistance - 2.5))) + 130;
      
    }
  }

  public void reBoot(){
      hingeEncoder.setPosition(0);
  }
}