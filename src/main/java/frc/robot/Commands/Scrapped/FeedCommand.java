package frc.robot.Commands.Scrapped;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedCommand extends Command{
    private ShooterSubsystem shoot;
    public FeedCommand(ShooterSubsystem shoot){
        this.shoot = shoot;
        addRequirements(shoot);
    }

    @Override
    public void execute(){
        shoot.feed();
        SmartDashboard.putBoolean("feeding", true);
    }

    @Override
    public void end(boolean isFinished){
        shoot.disableFeed();
        SmartDashboard.putBoolean("feeding", false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
