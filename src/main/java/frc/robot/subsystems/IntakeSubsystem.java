package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;

  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;

  public IntakeSubsystem() {
    m_motor = new CANSparkMax(Constants.Intake.kCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Intake.kPositionGains);

    m_motor.burnFlash();

    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = 0.0;
  }

  public void setPower(double _power) {
    if (!m_positionMode) {
      m_power = _power;
    }
  }

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

  @Override
  public void periodic() { // This method will be called once per scheduler run
    if (isNearTarget()) {
      m_positionMode = false;
      m_power = 0.0;
    }

    if (m_positionMode) {
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
    } else {
      m_motor.set(m_power);
    }
  }

  public boolean isNearTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition)
        < Constants.Intake.kPositionTolerance;
  }
}
