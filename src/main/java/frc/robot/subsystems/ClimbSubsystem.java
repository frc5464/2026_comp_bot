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
import frc.robot.Universals;

public class ClimbSubsystem extends SubsystemBase{  
  private final SparkMax climber = new SparkMax(55, MotorType.kBrushless);

  public RelativeEncoder climbEncoder;

  public double up = -10;
  public double down = 0;

  // private static final boolean ENABLED = true;

  public void initialize(){
    climbEncoder = climber.getEncoder();
    
    climbEncoder.setPosition(0);
  }

  public void periodic(){
    SmartDashboard.putNumber("climbEncoder", climbEncoder.getPosition());

    SmartDashboard.putBoolean("Auto Up", Universals.autoClimbUp);
    SmartDashboard.putBoolean("Auto Up", Universals.autoClimbDown);
  }

  // @Override
  // public boolean isEnabled(){
  //   return ENABLED;
  // }

  public void climbUp(){
      if(Universals.climbUp){
        if(Universals.climbOverride){
          climber.set(1);
        }
        else if(climbEncoder.getPosition() < up){
          climber.set(0);
        }
        else{
          climber.set(1);
        }
    }
    else{
      climbDisable();
    }
  }

  public void climbDown(){
      if(Universals.climbDown){
          if(Universals.climbOverride){
              climber.set(-1);
          }
          else if(climbEncoder.getPosition() > down){
              climber.set(0);
          }
          else{
              climber.set(-1);
          }
      }
      else{
          climbDisable();
      }
  }

  public void autoUp(){
      if(Universals.autoClimbUp){
          Universals.climbUp = true;
          climbUp();
      }
      else{
          climbDisable();
      }  
  }

  public void autoDown(){
      if(Universals.autoClimbDown){
          Universals.climbDown = true;
          climbDown();
      }
      else{
          climbDisable();
      }
  }

  public void climbDisable(){
    climber.set(0);
  }

  public void zeroEncoders(){
    climbEncoder.setPosition(0);
  }

    public void bringUp(){
      climber.set(1);
    }
    public void bringDown(){
      climber.set(-1);
    }
    // public void stop(){
    //   climber.set(0);
    // }

    // // public void reBoot(){
    // //   climbEncoder.setPosition(0);
    // // }
}
