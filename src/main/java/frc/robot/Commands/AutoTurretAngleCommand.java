package frc.robot.Commands;

import com.ctre.phoenix6.controls.DifferentialVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision;
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

    Pose2d smartPose(Pose2d lastPose, Pose2d visionPose2d, Pose2d drivetrainPose, double maxDeviationM,
            double maxDeviationD) {
        // preprocessing
        if (lastPose == null) {
            return null;
        }
        if (visionPose2d == Pose2d.kZero || visionPose2d == null) {
            return lastPose;
        }
        if (drivetrainPose == Pose2d.kZero || drivetrainPose == null) {
            return lastPose;
        }

        Pose2d deviation = new Pose2d(
                visionPose2d.getX() - drivetrainPose.getX(),
                visionPose2d.getY() - drivetrainPose.getX(),
                visionPose2d.getRotation().plus(drivetrainPose.getRotation())).div(2);
        
        if (deviation.getX() >maxDeviationM || deviation.getY() >maxDeviationM){
            //gulp
        }
        if (deviation.getRotation().getDegrees() > maxDeviationD){
            //gulp2
        }
        // processing
        return new Pose2d(
                (visionPose2d.getX() + drivetrainPose.getX() + deviation.getX()) / 2,
                (visionPose2d.getY() + drivetrainPose.getY() + deviation.getY()) / 2,
                new Rotation2d((deviation.getRotation().getDegrees() + visionPose2d.getRotation().getDegrees()
                        + drivetrainPose.getRotation().getDegrees()) / 2));
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
        double[] smbdPose = SmartDashboard.getNumberArray("visionInternalPose", new double[] {0,0,0});
        Pose2d visionPose = new Pose2d(smbdPose[0],smbdPose[1],new Rotation2d(smbdPose[2])); //TODO: check if radians or degrees
        Pose2d lastPose = new Pose2d(1,1,new Rotation2d(1));
        Pose2d pose = smartPose(lastPose, visionPose, drivetrain.getState().Pose, 10, 10);
        
        turret.autoAim(pose.getX(),pose.getY(),pose.getRotation().getDegrees());
        // turret.autoAim(drivetrain.getState().Pose.getX(),
                // drivetrain.getState().Pose.getY(),
                // drivetrain.getState().Pose.getRotation().getDegrees());

    }
}
