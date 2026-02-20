package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
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
        if(posDirection == 0){
            // intake.ManualRaiseIntake();
        } else if(posDirection == 1){
            // intake.ManualLowerIntake();
        }   else{
            // intake.stopElevate();
        }
    }
    
}
