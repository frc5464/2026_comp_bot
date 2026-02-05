package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUpCommand extends Command{
    
private final ClimbSubsystem climb;
    // private boolean m_climb;

    public ClimbUpCommand(ClimbSubsystem climb, boolean m_climb){
        this.climb = climb;
        // this.m_climb = m_climb;
    }

    @Override
    public void initialize() {
        // System.out.println("climbing?");
    }

    @Override
    public void execute() {
        // if(m_climb == true){
        //     if(climb.climbEncoderPos >= 0){
                // climb.bringUp();
        //     } else {
        //         climb.stop();
        //     }  
        // } else{
        //     climb.bringDown();
        // }

        SmartDashboard.putBoolean("Climbing", true);
    }

    @Override 
    public void end(boolean interrupted) {
        // climb.climbDisable();
        SmartDashboard.putBoolean("Climbing", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
