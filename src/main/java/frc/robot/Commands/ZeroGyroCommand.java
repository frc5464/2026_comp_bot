package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ZeroGyroCommand extends Command{
    private final Runnable reset;
    private CommandSwerveDrivetrain drivetrain;
    public ZeroGyroCommand(CommandSwerveDrivetrain commandSwerve){
        drivetrain = commandSwerve;
        reset = () -> {
            drivetrain.seedFieldCentric();
        };
    }

    @Override
    public void initialize(){
        // drivetrain.seedFieldCentric();
        reset.run();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

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
