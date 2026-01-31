package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;

public class SlowDriveModeCommand extends Command{
    
    @Override
    public void execute(){
        Universals.driveSpeedMultiplier = 0.5;
    }

    @Override
    public void end(boolean interrupted){
        Universals.driveSpeedMultiplier = 1;
    }

}
