package frc.robot.Commands.Scrapped;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooterCommand extends Command{

    private ShooterSubsystem shooter;

    public ReverseShooterCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
    }
    
    @Override
    public void execute(){
        // shooter.reverseShoot();
        SmartDashboard.putBoolean("RevShoot", true);
    }

    @Override
    public void end(boolean interrupted){
        // shooter.disableShoot();
        SmartDashboard.putBoolean("RevShoot", false);
    }
}
