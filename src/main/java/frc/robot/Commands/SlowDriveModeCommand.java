package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;

public class SlowDriveModeCommand extends Command{
    
    @Override
    public void execute(){
        Universals.driveSpeedMultiplier = 1.5;
        SmartDashboard.putBoolean("SlowMode", true);
    }

    @Override
    public void end(boolean interrupted){
        Universals.driveSpeedMultiplier = 3;
        SmartDashboard.putBoolean("SlowMode", false);
    }

    // @Override
    // public boolean isFinished(){
    //     return true;
    // }

}
