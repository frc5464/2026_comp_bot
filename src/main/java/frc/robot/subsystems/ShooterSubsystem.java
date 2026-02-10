package frc.robot.subsystems;
// package edu.wpi.first.wpilibj.examples.rapidreactcommandbot.subsystems;

// import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.examples.rapidreactcommandbot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase{

  private final SparkMax m_shooterMotor = new SparkMax(4, MotorType.kBrushless/*ShooterConstants.kShooterMotorPort*/);
  private final SparkMax m_feederMotor = new SparkMax(5, MotorType.kBrushless/*ShooterConstants.kFeederMotorPort*/);
  private final SparkMax shootRotator = new SparkMax(76, MotorType.kBrushless);

  RelativeEncoder flyEncoder;
  public double encoderVel;

  private SparkMaxConfig flyConfig = new SparkMaxConfig();
  private SparkClosedLoopController closedLoopController;

  public double targetPosition = 0;

  public ShooterSubsystem(){

  }

  private void initPidShoot(){
      closedLoopController = m_shooterMotor.getClosedLoopController();
      flyEncoder = m_shooterMotor.getEncoder();

      flyConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.1)
          .i(0)
          .d(0)
          .outputRange(-1, 1)
          .feedForward
              .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

              m_shooterMotor.configure(flyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)

      SmartDashboard.setDefaultNumber("ShootTargetPos", 0);

      
  }
  // private final Encoder m_shooterEncoder =
  //     new Encoder(
  //         ShooterConstants.kEncoderPorts[0],
  //         ShooterConstants.kEncoderPorts[1],
  //         ShooterConstants.kEncoderReversed);
  // private final SimpleMotorFeedforward m_shooterFeedforward =
  //     new SimpleMotorFeedforward(
  //         ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
  // private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0.0, 0.0);

  /** The shooter subsystem for the robot. */
  // public ShooterSubsystem() {
    // m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
    // m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);

    // Set default command to turn off both the shooter and feeder motors, and then idle
  //   setDefaultCommand(
  //       runOnce(
  //               () -> {
  //                 m_shooterMotor.disable();
  //                 m_feederMotor.disable();
  //               })
  //           .andThen(run(() -> {}))
  //           .withName("Idle"));
  // }

  // /**
  //  * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
  //  * up to the specified setpoint, and then runs the feeder motor.
  //  *
  //  * @param setpointRotationsPerSecond The desired shooter velocity
  //  */
  // public Command shootCommand(double setpointRotationsPerSecond) {
  //   return parallel(
  //           // Run the shooter flywheel at the desired setpoint using feedforward and feedback
  //           run(
  //               () -> {
  //                 m_shooterMotor.set(
  //                     m_shooterFeedforward.calculate(setpointRotationsPerSecond)
  //                         + m_shooterFeedback.calculate(
  //                             m_shooterEncoder.getRate(), setpointRotationsPerSecond));
  //               }),

  //           // Wait until the shooter has reached the setpoint, and then run the feeder
  //           waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_feederMotor.set(1)))
  //       .withName("Shoot");
  // }

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

  public void disableShoot(){
    m_shooterMotor.set(0);
  //   m_feederMotor.set(0);
  }

}
    
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
