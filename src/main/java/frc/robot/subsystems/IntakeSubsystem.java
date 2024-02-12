package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;

  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(Constants.Intake.kCanId, MotorType.kBrushless);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Intake.kPositionGains);

    m_motor.burnFlash();

    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = 0.0;
  }

  /**
   * Set the power to spin the motor at. This only applies outside of position mode.
   *
   * @param _power The power to apply to the motor (from -1.0 to 1.0).
   */
  public void setPower(double _power) {
    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = _power;
  }

  /**
   * Constructs a command that drives the rollers a specific distance (number of rotations) from the
   * current position and then ends the command.
   *
   * @return The retract command
   */
  public Command retract() {
    Command newCommand =
        new Command() {
          @Override
          public void initialize() {
            m_positionMode = true;
            m_targetPosition = m_encoder.getPosition() + Constants.Intake.kRetractDistance;
          }

          @Override
          public boolean isFinished() {
            return isNearTarget();
          }
        };

    newCommand.addRequirements(this);

    return newCommand;
  }

  /**
   * Constructs a command that feeds a note into the launcher by running the intake for a set amount
   * of time. This command takes control of the launcher subsystem to make sure the wheels keep
   * spinning during the launch sequence.
   *
   * @param _launcher The instance of the launcher subsystem
   * @return The launch command
   */
  public Command feedLauncher(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            setPower(1.0);
            _launcher.runLauncher();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShotFeedTime;
          }

          @Override
          public void end(boolean interrupted) {
            setPower(0.0);
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    // if we've reached the position target, drop out of position mode
    if (m_positionMode && isNearTarget()) {
      m_positionMode = false;
      m_power = 0.0;
    }

    // update the motor power based on mode and setpoint
    if (m_positionMode) {
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
    } else {
      m_motor.set(m_power);
    }
  }

  /**
   * Check if the encoder is within the position tolerance.
   *
   * @return Whether the position is within the tolerance.
   */
  public boolean isNearTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition)
        < Constants.Intake.kPositionTolerance;
  }
}
