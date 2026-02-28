package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterHoodCommand extends Command{
    
    private ShooterSubsystem shoot;
    private boolean up;

    public ShooterHoodCommand(ShooterSubsystem shoot, boolean up){
        this.shoot = shoot;
        this.up = up;
    }

    @Override
    public void execute(){
        if(up == true){
            shoot.targetPosition -= 0.25;
        } else{
            shoot.targetPosition += 0.25;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
