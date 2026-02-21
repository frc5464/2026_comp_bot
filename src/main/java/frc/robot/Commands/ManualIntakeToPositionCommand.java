package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeToPositionCommand extends Command{

    private IntakeSubsystem intake;
    public double posDirection;

    public ManualIntakeToPositionCommand(IntakeSubsystem intake, double posDirection){
        this.intake = intake;
        this.posDirection = posDirection;
    }

    @Override
    public void execute(){
        // if(Universals.manualMode == true){
        //     intake.ManualRaiseIntake();
        // } else {
        //     intake.ManualLowerIntake();
        // }
    }

    @Override
    public void end(boolean interrupted){
        // intake.stopElevate();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
