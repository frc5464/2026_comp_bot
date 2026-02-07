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
        intake.encoderPos = position;
        
    }

    @Override
    public void execute() {
        intake.jawPIDToLevel();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }




}
