package frc.robot.Commands;

// import java.lang.annotation.Target;

// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.Universals;
// import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{

    private ShooterSubsystem shooter;
    // private IntakeSubsystem intake;
    private BeltSubsystem belt;
    public Timer timer = new Timer();
    double time;
    public boolean reversed;

    public ShootCommand(ShooterSubsystem shooter, BeltSubsystem belt, boolean reversed, double time){
        this.shooter = shooter;
        this.belt = belt;
        // this.intake = intake;
        this.time = time;
        this.reversed = reversed;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        if(reversed == false){
            SmartDashboard.putBoolean("shooting", true);
            // double velocity = shooter.changeVel();
            shooter.shooterMotor.setControl(shooter.m_request.withVelocity(shooter.targetVelocity = -46));
            // shooter.shooterMotor.setControl(shooter.m_request.withVelocity(shooter.changeVel()));
            if((timer.get() >= 0.75)){
                shooter.feed();
                // intake.Intake();
                SmartDashboard.putBoolean("feeding", true);
                if(timer.get() >= 2){
                    belt.runBelt(-0.75);
                } else{
                    belt.runBelt(-1);
                }
                // SmartDashboard.putBoolean("intaking", true);
            }


        } else{
            shooter.reverseFeed();
            // shooter.targetVelocity = 50;
            }

    }

    @Override
    public void end(boolean interrupted){
        // shooter.targetVelocity = 0;
        shooter.shooterMotor.setControl(shooter.m_request.withVelocity(shooter.targetVelocity = 0));
        shooter.disableFeed();
        belt.stopBelt();
        // intake.DisableIntake();
        SmartDashboard.putBoolean("shooting", false);
        SmartDashboard.putBoolean("feeding", false);
        // SmartDashboard.putBoolean("intaking", false);

    }

    @Override
    public boolean isFinished(){
        // This should cause autonomous to only spit out game pieces for a bit
        if((timer.get() > time) && RobotState.isAutonomous()){
            return true;
        }
        return false;
    }
}
