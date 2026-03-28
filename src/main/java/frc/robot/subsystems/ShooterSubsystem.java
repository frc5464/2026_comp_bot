package frc.robot.subsystems;

// import java.util.Collection;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.jni.OrchestraJNI;
// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Universals;
// import frc.robot.Commands.ShootCommand;
// import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{

    public final TalonFX shooterMotor = new TalonFX(16/*ShooterConstants.kShooterMotorPort*/);
    public final TalonFX feederMotor = new TalonFX(17);
    public Orchestra m_orchestra = new Orchestra("output.chrp");

// Add a single device to the orchestra
// m_orchestra.addr

// // Attempt to load the chrp
// var status = m_Orchestra.loadMusic("output.chrp");

// if (!status.isok()){
//     // log error
// }
// }


    //Stuff for shootPosition PID
    // public RelativeEncoder hingeEncoder;
    // public double encoderPos;
    
    // private SparkMaxConfig posConfig = new SparkMaxConfig();
    // private SparkClosedLoopController posClosedLoopController;

    // public double targetHoodPos = -0.24;
    public double usualResistance = 1.0;
    //Stuff for shootVelocity PID
    RelativeEncoder flyEncoder;
    public double encoderVel;

    public SparkClosedLoopController flyClosedLoopController;

    public double targetVelocity = 0; // start out at roughly our max velocity

    // public double hoodlimitup = -4;
    // public double hoodlimitdown = -0.4;

    public VelocityVoltage m_request;

    public double distancetoHub;

  public ShooterSubsystem(){
    initPidShoot();
    SmartDashboard.putBoolean("shooting", false);
  }

  private void initPidShoot(){
      // Position PID for shoot hinge
      // posClosedLoopController = shootHinge.getClosedLoopController();
      // hingeEncoder = shootHinge.getEncoder();  
      // posConfig.closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     //Set PID values for position control
      //     .p(0.1)
      //     .i(0)
      //     .d(0)
      //     .outputRange(-0.3, 0.3)
      //     .feedForward
      //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);
      // shootHinge.configure(posConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

     //VelocityPID for shooter with Krakens
      var slot0Configs = new Slot0Configs();
      slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0; // no output for integrated error
      slot0Configs.kD = 0; // no output for error derivative

      shooterMotor.getConfigurator().apply(slot0Configs);
      m_request = new VelocityVoltage(targetVelocity).withSlot(0);
  }

  public void periodic(){
      // ============================================== FLYWHEEL VELOCITY CODE!
      // create a velocity closed-loop request, voltage output, slot 0 configs, then use it in setcontrol
      m_request = new VelocityVoltage(targetVelocity).withSlot(0);
      shooterMotor.setControl(m_request.withVelocity(targetVelocity).withFeedForward(0));
      encoderVel = shooterMotor.getVelocity().getValueAsDouble();
      
      // ============================================== ROTATION POSITION CODE!
      // encoderPos = hingeEncoder.getPosition();

      // posClosedLoopController.setSetpoint(targetHoodPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      // SmartDashboard.putNumber("targetHoodPos", targetHoodPos);
      // SmartDashboard.putNumber("hoodRot", encoderPos);
      SmartDashboard.putNumber("shootVel", encoderVel);
      SmartDashboard.putNumber("targetShootVel", targetVelocity);
      
    }
  
  public void feed(){
    feederMotor.set(1);
  }

  public void reverseFeed(){
    feederMotor.set(-0.5);
  }

  public void shoot(){
    shooterMotor.set(0.95);
  }

  public void disableShoot(){
    shooterMotor.set(0);
    // feederMotor.set(0);
  }

  public void disableFeed(){
    feederMotor.set(0);
  }

  public double changeVel(double xrobot, double yrobot){
    double turretCenterx;
    double turretCentery;

    turretCenterx = xrobot-0.0889;
    turretCentery = yrobot-0.17145;
    // // Figure out which hub we need to be aiming at
    if(DriverStation.getAlliance().get() == Alliance.Blue){
        // Calculate the angle needed between hood and blue hub in degrees
        distancetoHub = Math.hypot(turretCenterx-4.6, turretCentery-4);
    } else{
        // Calculate the angle needed between hood and red hub in degrees
        distancetoHub =  Math.hypot(turretCenterx-11.9, turretCentery-4);  
    }
      double calculatedVelocity = (7.69*distancetoHub) + 65.3;
      if(calculatedVelocity > 100){
        calculatedVelocity = 100;
      }
      SmartDashboard.putNumber("calcShootVel", calculatedVelocity);
      return calculatedVelocity;
  }

  public void changeAngle(double xrobot, double yrobot){

    // double turretCenterx;
    // double turretCentery;

    // turretCenterx = xrobot-0.0889;
    // turretCentery = yrobot-0.17145;

    // // Figure out which hub we need to be aiming at
    // if(DriverStation.getAlliance().get() == Alliance.Blue){
    //     // Calculate the angle needed between hood and blue hub in degrees
    //     distancetoHub = Math.hypot(turretCenterx-4.6, turretCentery-4);
    // } else{
    //     // Calculate the angle needed between hood and red hub in degrees
    //     distancetoHub =  Math.hypot(turretCenterx-11.9, turretCentery-4);  
    // }
    
    // double calcHoodPos = (-2.09*distancetoHub) + 2.89;

    // if(calcHoodPos < hoodlimitup){
    //   targetHoodPos = hoodlimitup;
    // }
    // else if(calcHoodPos > hoodlimitdown){
    //   targetHoodPos = hoodlimitdown;
    // }
    // else{
    //   targetHoodPos = calcHoodPos;
    // }

    // SmartDashboard.putNumber("distancetoHub", distancetoHub);
    // SmartDashboard.putNumber("calcHoodPos", calcHoodPos);
  }      

      
  //   if(shootHinge.getOutputCurrent() > usualResistance){
  //     shootHinge.set(0);
  //   } else {
  //   /* Make a Linear relationship between the distance value and the ShootHinge 
  //    * motor encoder. Tune for smaller #'s. Y^2 - Y^1 / X^2 - X^1 = M. 
  //    * Y^1 = MX^1 + b (Solve for b)
  //    */
  //   // y = mx + b
  //   // m = slope = (y2 - y1)/(x2 - x1)
  //     targetHoodPos = (-40*(Math.hypot(xdistance - 4, ydistance - 2.5))) + 130;
  //   }
  // }

  public void reBoot(){
      // hingeEncoder.setPosition(0);
  }
}