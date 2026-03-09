package frc.robot.Commands;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{

    private ShooterSubsystem shooter;
    private BeltSubsystem belt;
    public Timer timer = new Timer();
    double time;
    public boolean reversed;

    public ShootCommand(ShooterSubsystem shooter, BeltSubsystem belt, boolean reversed, double time){
        this.shooter = shooter;
        this.belt = belt;
        this.time = time;
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
            // shoot at whatever our targetVelocity is!
            shooter.shooterMotor.setControl(shooter.m_request.withVelocity(shooter.targetVelocity));
            if((timer.get() >= 0.75)){
                shooter.feed();
                belt.runBelt();
                SmartDashboard.putBoolean("feeding", true);
            }
        } else{
            shooter.reverseFeed();
        }
    }

    @Override
    public void end(boolean interrupted){
        // Disable the motor by shorting out the neutral
        shooter.shooterMotor.setControl(new NeutralOut());
        shooter.disableFeed();
        belt.stopBelt();
        SmartDashboard.putBoolean("shooting", false);
        SmartDashboard.putBoolean("feeding", false);
    }

    @Override
    public boolean isFinished(){
        // This should cause autonomous to only spit out game pieces for a bit
        if((timer.get() > time) && RobotState.isAutonomous()){
            return true;
        }
        return false;
    }
}
