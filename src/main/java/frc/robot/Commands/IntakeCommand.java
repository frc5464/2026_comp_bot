package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Universals;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
            intake.Intake();
        SmartDashboard.putBoolean("intaking", true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.DisableIntake();
        SmartDashboard.putBoolean("intaking", false);
    }

    @Override
    public boolean isFinished(){
            return false;
    }
}
