package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroIntakeCommand extends Command{
    
    private IntakeSubsystem intake;

    public ZeroIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        intake.reBoot();
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
