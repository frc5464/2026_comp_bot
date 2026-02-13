package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbToPositionCommand extends Command{
    
    private ClimbSubsystem climb;

    private double position;

    public ClimbToPositionCommand(ClimbSubsystem climb, double pos){
        this.climb = climb;
        this. position = pos;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // if(position == 0){
        //     climb.targetPosition = 0;
        // }
        // else if(position == 1){
        //     climb.targetPosition = 1;
        // }
        // else{
        //     intake.stopElevate();
        // }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}