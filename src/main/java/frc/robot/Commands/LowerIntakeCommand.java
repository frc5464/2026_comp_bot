package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class LowerIntakeCommand extends Command{
    
private final IntakeSubsystem intake;

    public LowerIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        intake.ManualLowerIntake();
    }

    @Override
    public void end(boolean interrupted){
        intake.stopElevate();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
