package frc.robot.Commands;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{

    private ShooterSubsystem shooter;
    private BeltSubsystem belt;
    private Timer timer = new Timer();
    public boolean reversed;

    public ShootCommand(ShooterSubsystem shooter, boolean reversed){
        this.shooter = shooter;
        this.belt = belt;
        this.reversed = reversed;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        if(reversed == false){
            SmartDashboard.putBoolean("shooting", true);
                // shooter.shooterMotor.setControl(shooter.m_request.withVelocity(shooter.targetVelocity).withFeedForward(0));
            shooter.targetVelocity = -110;
            if(shooter.encoderVel <= -100){
                shooter.feed();
                belt.runBelt();
                SmartDashboard.putBoolean("UpToSpeed", true);
            } else{SmartDashboard.putBoolean("UpToSpeed", false);} // JAKEREVIEW: You are printing true and false here forever.
        } else{
            shooter.targetVelocity = 50;
            }

    }

    @Override
    public void end(boolean interrupted){
        shooter.targetVelocity = 0;
        shooter.disableShoot();
        // belt.stopBelt();
        SmartDashboard.putBoolean("shooting", false);

    }

    @Override
    public boolean isFinished(){
        // This should cause autonomous to only spit out game pieces for a bit
        if((timer.get() > 3) && RobotState.isAutonomous()){
            return true;
        }
        return false;
    }
}
