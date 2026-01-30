package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    private final SparkMax turret = new SparkMax(53, MotorType.kBrushless);
    private final SparkMax shooter = new SparkMax(54, MotorType.kBrushless);
    private final SparkMax feederer = new SparkMax(41, null);
    private final SparkMax raiseShoot = new SparkMax(67, MotorType.kBrushless);
    
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
                  shooter.disable();
                  feederer.disable();
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
                  shooter.set(
                      m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + m_shooterFeedback.calculate(
                              m_shooterEncoder.getRate(), setpointRotationsPerSecond));
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_feederMotor.set(1)))
        .withName("Shoot");
  }
}
