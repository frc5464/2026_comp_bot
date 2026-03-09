package frc.robot.Commands.Scrapped;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedCommand extends Command{
    private ShooterSubsystem shoot;
    // private BeltSubsystem belt;
    Timer timer = new Timer();
    double time;
    boolean reversed;
    
    public FeedCommand(ShooterSubsystem shoot, BeltSubsystem belt, Boolean reversed, double time){
        this.shoot = shoot;
        // this.belt = belt;
        this.reversed = reversed;
        this.time = time;
        addRequirements(shoot, belt);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        if(reversed == false){
            shoot.feed();
        } else{
            shoot.reverseFeed();
        }
        
        // belt.runBelt();
        // SmartDashboard.putBoolean("feeding", true);
    }

    @Override
    public void end(boolean isFinished){
        shoot.disableFeed();
        // belt.stopBelt();
        // SmartDashboard.putBoolean("feeding", false);
    }

    @Override
    public boolean isFinished(){
        if((timer.get() > time) && RobotState.isAutonomous()){
            return true;
        }
        return false;
    }
}
