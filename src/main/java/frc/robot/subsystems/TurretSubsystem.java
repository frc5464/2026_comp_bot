package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
// import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
import frc.robot.Universals;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax turret = new SparkMax(14, MotorType.kBrushless);
    // private final SparkMaxSim turretsim = new SparkMaxSim(turret, null);
    public RelativeEncoder turretEncoder;
    public double turretEncoderPos;
    public double angletoHub;

    public double turretAngle;
    
    private SparkMaxConfig turretConfig = new SparkMaxConfig();
    private SparkClosedLoopController turretClosedLoopController;

    public double turrettargetPosition = 0;

    public double turretlimitleft = 18;
    public double turretlimitright = -15.5;

    public boolean turretAimedtoShoot = false;

    public double gear_ratio = 64;
    public double degrees_per_rotation = 5.625;

    public TurretSubsystem(){
        initPidTurret();         

    }

    private void initPidTurret(){
        // do your pid initialization here
        turretClosedLoopController = turret.getClosedLoopController();
        turretEncoder = turret.getEncoder();

        
        turretConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //Set PID values for position control
            .p(0.075)
            .i(0)
            .d(0)
            .outputRange(-0.2, 0.2)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                turretConfig.inverted(true);
                turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // SmartDashboard.setDefaultNumber("turret target Position", 0);
    }



    public void periodic(){
        if(Universals.manualMode == false){
            turretEncoderPos = turretEncoder.getPosition();
            turretClosedLoopController.setSetpoint(turrettargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
        turretEncoderPos = turretEncoder.getPosition();
        SmartDashboard.putNumber("turretEncoder", turretEncoderPos);
    }

    public void reBoot(){
        turretEncoder.setPosition(0);
    }

    public void clockwise(){
        if(turretEncoderPos > turretlimitleft) {
            stop();
        } else{
            turret.set(-0.2);
        }
    }

    public void counterclockwise(){
        if(turretEncoderPos < turretlimitright){
            stop();
        } else{
            turret.set(0.2);
        }
    }

    public void stop(){
        turret.set(0);
    }

    public void autoAim(double xrobot, double yrobot, double heading){


        double turretCenterx;
        double turretCentery;

        double calculatedPosition;
        double calculatedTurretAngle;

        turretCenterx = xrobot-0.0889;
        turretCentery = yrobot-0.17145;

        // Figure out which hub we need to be aiming at
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            // Calculate the angle needed between turret and blue hub in degrees
            angletoHub =  -Math.toDegrees(Math.atan((turretCentery-4)/(turretCenterx-5.4)));
        }
        else{
            // Calculate the angle needed between turret and red hub in degrees
            angletoHub =  Math.toDegrees(Math.atan((turretCenterx-11.6)/(turretCentery-4))); 
            
            // normalize the value to be 0 when facing the turret
            if(angletoHub >= 0){
                angletoHub -= 90;
            }
            else{
                angletoHub += 90;
            }
            
            // swap heading if we are on the red team
            if(heading > 0){
                heading -= 180;
            }
            else{
                heading += 180;
            }
        }

        // Add the chassis angle to the angle to the hub
        calculatedTurretAngle = heading + angletoHub;   

        // left rotations = more positive
        // right rotations = more negative
        // for both sides, calcTurretAngle going positive = too far left.
        // If chassis is aimed left, then aim the turret right.
        // That means we must invert the value that we send to the control loop.
        calculatedPosition = -(calculatedTurretAngle / degrees_per_rotation);

        // always shoot straight forward if not in our scoring zone
        if((xrobot > 5) && xrobot < 11.4){
            turrettargetPosition = 0;
            turretAimedtoShoot = true;
        }
        // keep our values within safe limits if in scoring zone
        else if(calculatedPosition > turretlimitleft){
            turrettargetPosition = turretlimitleft;
            turretAimedtoShoot = false;
        }
        else if(calculatedPosition < turretlimitright){
            turrettargetPosition = turretlimitright;
            turretAimedtoShoot = false;
        } else{
            turretAimedtoShoot = true;
            turrettargetPosition = calculatedPosition;
        }

        SmartDashboard.putNumber("angletoHub", angletoHub);
        SmartDashboard.putNumber("heading", heading);
        SmartDashboard.putNumber("calcTurretAngle", calculatedTurretAngle);
        SmartDashboard.putNumber("calcTurretPos", calculatedPosition);
        SmartDashboard.putNumber("TurretTarget", turrettargetPosition);
        SmartDashboard.putBoolean("TurretAimed2Shoot", turretAimedtoShoot);
    }
}