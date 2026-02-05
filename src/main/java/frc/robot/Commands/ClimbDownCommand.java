package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDownCommand extends Command{
    
    private final ClimbSubsystem climb;

    public ClimbDownCommand(ClimbSubsystem climb){
        this.climb = climb;
    }

    @Override
    public void execute(){
        // climb.climbDown();
        SmartDashboard.putBoolean("Descending", true);
    }

    @Override
    public void end(boolean interrupted){
        // climb.climbDisable();
    }
}
