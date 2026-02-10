package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{

    private ShooterSubsystem shooter;

    private double velocity;

    public ShootCommand(ShooterSubsystem shooter, double velocity){
        this.shooter = shooter;
        this.velocity = velocity;
    }

    @Override
    public void execute(){
        shooter.revUp();
        if(velocity >= 200){
            shooter.feed();
            SmartDashboard.putBoolean("UpToSpeed", true);
        } else{
            shooter.disableShoot();
            SmartDashboard.putBoolean("UpToSpeed", false);
        }
        SmartDashboard.putBoolean("shooting", true);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("shooting", false);

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
