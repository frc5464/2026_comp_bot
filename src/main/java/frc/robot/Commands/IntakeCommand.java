package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private final IntakeSubsystem intake;
    private boolean m_intake;

    private Timer timer = new Timer();

    public IntakeCommand(IntakeSubsystem intake, boolean m_intake){
        this.intake = intake;
        this.m_intake = m_intake;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(m_intake == true){
            intake.intake();
        } else{
            intake.outake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        // if(wrist.getIntOutCurrent() > 38){
        //     return true;
        // }

        // This should cause autonomous to only spit out game pieces for a bit
        if((timer.get() > 0.5) && RobotState.isAutonomous() && (!m_intake)){
            return true;
        }

         // This should cause autonomous to only intake game pieces for a bit
        else if((timer.get() > 1.5) && RobotState.isAutonomous() && m_intake){
            return true;
        }
        
        return false;
    }

}
