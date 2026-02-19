package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{

    private ShooterSubsystem shooter;
    private BeltSubsystem belt;
    private Timer timer = new Timer();

    public ShootCommand(ShooterSubsystem shooter, BeltSubsystem belt){
        this.shooter = shooter;
        this.belt = belt;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        // shooter.targetVelocity = 200;
        // if(shooter.encoderVel >= 200){
            // shooter.feed();
            // belt.runBelt();
            // SmartDashboard.putBoolean("UpToSpeed", true);   // JAKEREVIEW: You are printing true and false here forever.
        // } else{
            // SmartDashboard.putBoolean("UpToSpeed", false);
        // }
        SmartDashboard.putBoolean("shooting", true);
    }

    @Override
    public void end(boolean interrupted){
        // shooter.targetVelocity = 0;
        // shooter.disableShoot();
        // belt.stopBelt();
        SmartDashboard.putBoolean("shooting", false);

    }

    @Override
    public boolean isFinished(){
        // This should cause autonomous to only spit out game pieces for a bit
        if((timer.get() > 2) && RobotState.isAutonomous()){
            return true;
        }
        return false;
    }
}
