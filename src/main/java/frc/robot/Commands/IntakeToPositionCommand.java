package frc.robot.Commands;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

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
            intake.targetPositionInt = -0.1;
        }
        else if(position == 1){
            intake.targetPositionInt = -3;
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
