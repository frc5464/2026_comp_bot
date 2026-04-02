package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class AutoTurretAngleCommand extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private TurretSubsystem turret;

    double[] dashboardVision;

    public AutoTurretAngleCommand(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        /*
         * dashboardVision = SmartDashboard.getNumberArray("robopose", new double[] { 0,
         * 0, 0 });
         * if (dashboardVision == new double[] { 0, 0, 0 }) {
         * turret.autoAim(drivetrain.getState().Pose.getX(),
         * drivetrain.getState().Pose.getY(),
         * drivetrain.getState().Pose.getRotation().getDegrees());
         * } else {
         * turret.autoAim(
         * dashboardVision[0],
         * dashboardVision[1],
         * dashboardVision[2]);
         * }
         */

        turret.autoAim(drivetrain.getState().Pose.getX(),
                drivetrain.getState().Pose.getY(),
                drivetrain.getState().Pose.getRotation().getDegrees());

    }
}
