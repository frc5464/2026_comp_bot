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

    // private Timer timer = new Timer();

    public IntakeCommand(IntakeSubsystem intake/*, boolean m_intake*/){
        this.intake = intake;
        // this.m_intake = m_intake;
    }

    @Override
    public void initialize() {
        
        // if(RobotState.isAutonomous()){
        //     Universals.homingPathToFuel = true;
        // timer.reset();
        // timer.start();
        // }
    }

    @Override
    public void execute() {
        // if(Universals.intaking == true){
            intake.Intake();
        // return;
        // }
        SmartDashboard.putBoolean("intaking", true);
        // System.out.print("intaking!!!!");
    }

    @Override
    public void end(boolean interrupted) {
        // Universals.intaking = false;
        // if(RobotState.isAutonomous()){
        //     Universals.homingPathToFuel = false;
        // }
        intake.DisableIntake();
        SmartDashboard.putBoolean("intaking", false);
    }

    //Returns value of boolean and quits the command when break beam sensor is tripped

    @Override
    public boolean isFinished(){
        // if (intake.fuelnotdetected == false) {
        //     // Universals.LEDselected = "PinkIntake";
        //     return true;
        // }
        // else {
            return false;
        // }
    }

    // @Override
    // public boolean isFinished() {
    //     // if(wrist.getIntOutCurrent() > 38){
    //     //     return true;
    //     // }

    //     // This should cause autonomous to only spit out game pieces for a bit
    //     if((timer.get() > 0.5) && RobotState.isAutonomous() && (!m_intake)){
    //         return true;
    //     }

    //      // This should cause autonomous to only intake game pieces for a bit
    //     else if((timer.get() > 1.5) && RobotState.isAutonomous() && m_intake){
    //         return true;
    //     }
        
    //     return false;
    // }

}
