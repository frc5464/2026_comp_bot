package frc.robot.Commands.Scrapped;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Universals;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeCommand extends Command{
    private final IntakeSubsystem intake;
    private Timer timer = new Timer();

    public ReverseIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        // timer.reset();
        // timer.start();
    }

    @Override
    public void execute() {
            intake.IntakeReverse();
        SmartDashboard.putBoolean("-intaking", true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.DisableIntake();
        SmartDashboard.putBoolean("-intaking", false);
    }
}