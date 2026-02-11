package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToPositionCommand extends Command{
    
    private IntakeSubsystem intake;

    private double position;

    public IntakeToPositionCommand(IntakeSubsystem intake, double pos){
        this.intake = intake;
        this. position = pos;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(position == 0){
            intake.targetPosition = -0.10;
        }
        else if(position == 1){
            intake.targetPosition = -3.5;
        }
        else{
            intake.stopElevate();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }




}
