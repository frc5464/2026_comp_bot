package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltSubsystem;

public class BeltCommand extends Command{
    
    private BeltSubsystem belt;

    public BeltCommand(BeltSubsystem belt){
        this.belt = belt;
        addRequirements(belt);
    }

    @Override
    public void execute(){
        belt.runBelt();
        SmartDashboard.putBoolean("Belting", true);
    }

    @Override
    public void end(boolean interrupted){
        belt.stopBelt();
        SmartDashboard.putBoolean("Belting", false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
