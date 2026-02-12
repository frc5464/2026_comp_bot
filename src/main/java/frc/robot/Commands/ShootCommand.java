package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{

    private ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute(){
        // shooter.targetVelocity = 200;
        // if(shooter.encoderVel >= 200){
        //     shooter.feed();
            SmartDashboard.putBoolean("UpToSpeed", true);
        // } else{
            SmartDashboard.putBoolean("UpToSpeed", false);
        // }
        // SmartDashboard.putBoolean("shooting", true);
    }

    @Override
    public void end(boolean interrupted){
        // shooter.targetVelocity = 0;
        // shooter.disableShoot();
        SmartDashboard.putBoolean("shooting", false);

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
