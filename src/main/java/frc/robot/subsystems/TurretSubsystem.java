package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Universals;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax turret = new SparkMax(14, MotorType.kBrushless);

    public RelativeEncoder turretEncoder;
    public double turretEncoderPos;
    public double angletoHub;

    public double turretAngle;
    
    private SparkMaxConfig turretConfig = new SparkMaxConfig();
    private SparkClosedLoopController turretClosedLoopController;

    public double turrettargetPosition = 0;

    public double turretlimitleft = 22.7;
    public double turretlimitright = -22.8;

    public double gear_ratio = 192;
    public double degrees_per_rotation = 1.875;

    private CommandSwerveDrivetrain drivetrain;


    public TurretSubsystem(CommandSwerveDrivetrain drivetrain){

        this.drivetrain = drivetrain;
        initPidTurret();         

    }

    private void initPidTurret(){
        // do your pid initialization here
        turretClosedLoopController = turret.getClosedLoopController();
        turretEncoder = turret.getEncoder();

        
        turretConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-0.3, 0.3)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("turret target Position", 0);
    }



    public void periodic(){
        if(Universals.manualMode == false){
            // turretEncoderPos = turretEncoder.getPosition();
            // turretClosedLoopController.setSetpoint(turrettargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
        turretEncoderPos = turretEncoder.getPosition();
        // turretClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("turretEncoder", turretEncoderPos);
        SmartDashboard.putNumber("TurretTarget", turrettargetPosition);

        // do your pid calculation here (use targetPosition!)
    }

    public void reBoot(){
        turretEncoder.setPosition(0);
    }

    public void clockwise(){
        if(turretEncoderPos > turretlimitleft) {
            stop();
        } else{
            turret.set(0.2);
        }
    }

    public void counterclockwise(){
        if(turretEncoderPos < turretlimitright){
            stop();
        } else{
            turret.set(-0.2);
        }
    }

    public void stop(){
        turret.set(0);
    }

    public void findTargetRotations(double xrobot, double yrobot){
        
        
        // find needed angle of turret to hub

        // find the angle of the robot to the hub

        // subtract out the robot angle

        // convert that angle into motor rotations

        // use that motor rotation as target
    }

    public void autoAim(double xrobot, double yrobot, double heading){
        xrobot = drivetrain.getState().Pose.getX();
        yrobot = drivetrain.getState().Pose.getY();
        heading = drivetrain.getState().Pose.getRotation().getDegrees();

        double turretCenterx;
        double turretCentery;

        double calculatedPosition = turrettargetPosition;
        
        turretCenterx = xrobot-3.5;
        turretCentery = yrobot-6.75;

        // Figure out which hub we need to be aiming at
        if(DriverStation.getAlliance().get() == Alliance.Blue){
        //     // Calculate the angle needed between turret and blue hub in degrees
            angletoHub =  Math.toDegrees(Math.atan((turretCentery-4)/(turretCenterx-4.6)));
        }
        else{
        //     // Calculate the angle needed between turret and red hub in degrees
            angletoHub =  Math.toDegrees(Math.atan((turretCenterx-11.9)/(turretCentery-4)));            
        }

        if(heading > 0){
            calculatedPosition = angletoHub + heading;
        } else if(heading < 0){
            calculatedPosition = angletoHub - heading;
        } else{
            calculatedPosition = turrettargetPosition;
        }

        // if(calculatedPosition < turretlimitleft){
        // turrettargetPosition = turretlimitleft;
        // }
        // else if(calculatedPosition > turretlimitright){
        // turrettargetPosition = turretlimitright;
        // } else{
        // turrettargetPosition = calculatedPosition;
        // }

        SmartDashboard.putNumber("angletoHub", angletoHub);
    }
}