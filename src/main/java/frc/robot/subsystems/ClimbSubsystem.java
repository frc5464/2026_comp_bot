package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    
    private final SparkMax climber = new SparkMax(55, MotorType.kBrushless);

    // private final SparkMax rightClimbMotor = new SparkMax(56, MotorType.kBrushless);
public RelativeEncoder climbEncoder;



  private static final boolean ENABLED = true;


  SparkMaxConfig sparkMaxConfig3 = new SparkMaxConfig();
  // SparkClosedLoopController loopController = climb1.getClosedLoopController();
  SparkClosedLoopController climbPID = climber.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = 0;
  double extMinOutput = 0;
  


  public double climbEncoderPos;
  public double counts;
  
  public ClimbSubsystem(){
    climbEncoder = climber.getEncoder();
    climbEncoder.setPosition(0);
  }

  public void periodic(){

    SmartDashboard.putNumber("ClimbEncoder", climbEncoderPos);
    //ClimbToLevel(0);
        //loopController.setReference(400, ControlType.kPosition );
        
        climbEncoderPos = climbEncoder.getPosition();
    } 
    
    // @Override
    // public boolean isEnabled() {
    //   return ENABLED;
    // }

    public void bringUp(){
      climber.set(1);
    }
    public void bringDown(){
      climber.set(-1);
    }
    public void stop(){
      climber.set(0);
    }

    public void reBoot(){
      climbEncoder.setPosition(0);
    }
}
