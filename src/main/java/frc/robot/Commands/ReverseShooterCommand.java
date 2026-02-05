package frc.robot.Commands;

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
    }

    @Override
    public void end(boolean interrupted){
        // shooter.disableShoot();
        
    }
}
