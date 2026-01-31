package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private final ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute(){
        shooter.shootCommand(0);
    }

    @Override
    public void end(boolean interrupted){
        shooter
        .disableShoot();
    }
}
