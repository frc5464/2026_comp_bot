package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Universals;
import frc.robot.subsystems.IntakeSubsystem;

public class LongIntakeCommand extends Command{
    private final IntakeSubsystem intake;
    private Timer timer = new Timer();

    public LongIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
            intake.Intake();
        SmartDashboard.putBoolean("longintaking", true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.DisableIntake();
        SmartDashboard.putBoolean("longintaking", false);
    }

    @Override
    public boolean isFinished(){
        if((timer.get() > 18) && RobotState.isAutonomous()){
            return true;
        }
            return false;
    }
}
