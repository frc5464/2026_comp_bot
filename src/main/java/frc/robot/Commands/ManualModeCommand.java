package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;

public class ManualModeCommand extends Command{
    @Override
    public void initialize() {
        Universals.manualMode = true;
    }

    @Override
    public void end(boolean interrupted) {
        Universals.manualMode = false;
        System.out.println("manualModeCommand_fin");
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
    
}
