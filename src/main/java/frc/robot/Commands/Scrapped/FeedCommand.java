package frc.robot.Commands.Scrapped;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedCommand extends Command{
    private ShooterSubsystem shoot;
    Timer timer = new Timer();
    double time;
    public FeedCommand(ShooterSubsystem shoot, double time){
        this.shoot = shoot;
        this.time = time;
        addRequirements(shoot);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
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
        if((timer.get() > time) && RobotState.isAutonomous()){
            return true;
        }
        return false;
    }
}
