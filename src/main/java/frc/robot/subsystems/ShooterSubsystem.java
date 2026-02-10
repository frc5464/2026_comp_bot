package frc.robot.subsystems;
// package edu.wpi.first.wpilibj.examples.rapidreactcommandbot.subsystems;

// import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.Set;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.examples.rapidreactcommandbot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// <<<<<<< HEAD
// public class ShooterSubsystem extends SubsystemBase{
    
//     private final SparkMax turret = new SparkMax(53, MotorType.kBrushless);
//     private final SparkMax shooter = new SparkMax(54, MotorType.kBrushless);

//     public Command runShooter(){
//         return runOnce(
//             () -> {
//                 shooter.set(1);
//             });
//     }

//     public Command turret(){
//         return run(
//             () -> {
//                 /* Auto target an april tag attatched to the hub */
//             });
//     }
// =======
@Logged
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_shooterMotor = new SparkMax(4, MotorType.kBrushless/*ShooterConstants.kShooterMotorPort*/);
  private final SparkMax m_feederMotor = new SparkMax(5, MotorType.kBrushless/*ShooterConstants.kFeederMotorPort*/);
  private final SparkMax shootRotator = new SparkMax(76, MotorType.kBrushless);

  private final Encoder m_shooterEncoder =
      new Encoder(
          ShooterConstants.kEncoderPorts[0],
          ShooterConstants.kEncoderPorts[1],
          ShooterConstants.kEncoderReversed);
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
  private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0.0, 0.0);

  /** The shooter subsystem for the robot. */
  public ShooterSubsystem() {
    m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);

    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  m_shooterMotor.disable();
                  m_feederMotor.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */
  public Command shootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  m_shooterMotor.set(
                      m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + m_shooterFeedback.calculate(
                              m_shooterEncoder.getRate(), setpointRotationsPerSecond));
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_feederMotor.set(1)))
        .withName("Shoot");
  }

  // public void liftShooter(){
  //   shootRotator.set(1);
  // }

  // public void lowerShooter(){
  //   shootRotator.set(-1);
  // }

  // public void shooterNoMove(){
  //   shootRotator.set(0);
  // }

  // public void reverseShoot(){
  //   m_shooterMotor.set(-1);
  //   m_feederMotor.set(-1);
  // }

  // public void disableShoot(){
  //   m_shooterMotor.set(0);
  //   m_feederMotor.set(0);
  // }





}



// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WrapperCommand;
// import frc.robot.Universals;

// public class ShooterSubsystem extends SubsystemBase{
    
//     private final SparkMax turret = new SparkMax(53, MotorType.kBrushless);
//     private final SparkMax shooter = new SparkMax(54, MotorType.kBrushless);
//     private final SparkMax feederer = new SparkMax(41, null);
//     private final SparkMax raiseShoot = new SparkMax(67, MotorType.kBrushless);
    
//     public RelativeEncoder shootRelativeEncoder = shooter.getEncoder();

//     PIDController pid;

//     // private double kp = 0;
//     // private double ki = 0;
//     // private double kd = 0;

//     double kP_top, kI_top, kD_top, kIz_top, kFF_top, kMaxOutput_top, kMinOutput_top, maxRPM_top;
//     double kP_bottom, kI_bottom, kD_bottom, kIz_bottom, kFF_bottom, kMaxOutput_bottom, kMinOutput_bottom, maxRPM_bottom;

//     public double shoothub;

//     public double shootdefault;

//     public double FullSpeedShoot;

//     public double hubRPM;

//     public double hubPitch;

//     public double shootRPMselect = 0;

//     public double shootConstants;

//     private static final boolean ENABLED = true;

//     // @Override
//     public boolean isEnabled(){
//       return ENABLED;
//     }

//     // @Override
//     public void initialize(){
//       kP_bottom = 0.00012;
//         kI_bottom = 0.00000001;
//         kD_bottom = 0.0;
//         kIz_bottom = 0;
//         kFF_bottom = 0.00018;
//         kMaxOutput_bottom = 1;
//         kMinOutput_bottom = -1;
//         maxRPM_bottom = 5700;

//         kP_top = 0.00012;
//         kI_top = 0.00000001;
//         kD_top = 0.0;
//         kIz_top = 0;
//         kFF_top = 0.00018;
//         kMaxOutput_top = 1;
//         kMinOutput_top = -1;
//         maxRPM_top = 5700;

//         pid.setP(kP_bottom);
//         pid.setI(kI_bottom);
//         pid.setD(kD_bottom);
//         pid.setIZone(kIz_bottom);
//         pid.setPID();
//         pid.setOutputRange(kMinOutput_bottom, kMaxOutput_bottom);

//         // PIDTop.setP(kP_top);
//         // PIDTop.setI(kI_top);
//         // PIDTop.setD(kD_top);
//         // PIDTop.setIZone(kIz_top);
//         // PIDTop.setFF(kFF_top);
//         // PIDTop.setOutputRange(kMinOutput_top, kMaxOutput_top);
//         // shootTop.setIdleMode(IdleMode.kBrake);
//     }

//     public void Homing(double SPtop, double SPbottom){
//         PIDTop.setReference(SPtop, CANSparkMax.ControlType.kVelocity);
//         PIDBottom.setReference(SPbottom, CANSparkMax.ControlType.kVelocity);
//     }

//     public void periodic(){
//       // shooter.set(pid.calculate(m_shooterEncoder.getDistance()), 0);
//       SmartDashboard.putBoolean("up to speed!", Universals.shootUptoSpeed);

//       SmartDashboard.putNumber("shoot encoder", shootRelativeEncoder.getVelocity());
    
//     }

//     public void shootCommand(){
//       FullSpeedShoot = shootRelativeEncoder.getVelocity();
//       if ((FullSpeedShoot > (shootRPMselect))) {
//         Universals.shootUptoSpeed = true;
//       }
//       else{
//         DisableShoot();
//       }
//     }

//     public void ShootReverse(){
//       feederer.set(-1);
//     }

//     public void DisableShoot(){
//       shooter.set(0);
//     }


    
//     // private final Encoder m_shooterEncoder =
//     //   new Encoder(0, 0, 0
//           // ShooterConstants.kEncoderPorts[0],
//           // ShooterConstants.kEncoderPorts[1],
//           // ShooterConstants.kEncoderReversed
//       // );
//   // private final SimpleMotorFeedforward m_shooterFeedforward =
//   //     new SimpleMotorFeedforward(
//   //         ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
//   // private final PIDController m_shooterFeedback = new PIDController(Universals.shootSpeedMultiplier, 0.0, 0.0);

//   // /** The shooter subsystem for the robot. */
//   // public ShooterSubsystem() {
//   //   m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
//   //   m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    
//   //   shooter.getAbsoluteEncoder();

//   //   feederer.getAbsoluteEncoder();
//   //   // Set default command to turn off both the shooter and feeder motors, and then idle
//   //   setDefaultCommand(
//   //       runOnce(
//   //               () -> {
//   //                 shooter.disable();
//   //                 feederer.disable();
//   //               })
//   //           .andThen(run(() -> {}))
//   //           .withName("Idle"));
//   // }

//   // /**
//   //  * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
//   //  * up to the specified setpoint, and then runs the feeder motor.
//   //  *
//   //  * @param setpointRotationsPerSecond The desired shooter velocity
//   //  */
//   // public Command shootCommand(double setpointRotationsPerSecond) {
//   //   return parallel(
//   //           // Run the shooter flywheel at the desired setpoint using feedforward and feedback
//   //           run(
//   //               () -> {
//   //                 shooter.set(
//   //                     m_shooterFeedforward.calculate(setpointRotationsPerSecond)
//   //                         + m_shooterFeedback.calculate(
//   //                             m_shooterEncoder.getRate(), setpointRotationsPerSecond));
//   //               }),

//   //           // Wait until the shooter has reached the setpoint, and then run the feeder
//   //           waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_feederMotor.set(1)))
//   //       .withName("Shoot");
//   // }
// }
