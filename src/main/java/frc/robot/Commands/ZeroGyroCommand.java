package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ZeroGyroCommand extends Command{
    private final Runnable reset;
    private CommandSwerveDrivetrain drivetrain;
    public ZeroGyroCommand(CommandSwerveDrivetrain commandSwerve){
        drivetrain = commandSwerve;
        reset = () -> {
            // drivetrain.seedFieldCentric();
            drivetrain.seedFieldCentric180();
        };
    }

    @Override
    public void initialize(){
        // drivetrain.seedFieldCentric();
        SmartDashboard.putBoolean("zeroGyro", true);
        System.out.println("zeroed");
        reset.run();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("zeroGyro", false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
