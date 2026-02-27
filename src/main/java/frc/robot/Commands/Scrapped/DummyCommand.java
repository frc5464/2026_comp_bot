package frc.robot.Commands.Scrapped;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DummyCommand extends Command{
    
    public DummyCommand(){

    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("dummying", true);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("dummying", false);
    }
}
