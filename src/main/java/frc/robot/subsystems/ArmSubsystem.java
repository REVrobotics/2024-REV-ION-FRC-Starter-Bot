// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new CANSparkMax(Constants.Arm.kArmCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kSoftLimitForward);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kSoftLimitReverse);

    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    m_encoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Arm.kArmPositionGains);

    m_motor.burnFlash();

    m_setpoint = Constants.Arm.kHomePosition;

    m_timer = new Timer();
    m_timer.start();

    updateMotionProfile();
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    m_startState = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
    m_timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    m_controller.setReference(
        m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  public void runManual(double _power) {
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
    // passively
    m_setpoint = m_encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    m_motor.set(_power + (m_feedforward / 12.0));
    m_manualValue = _power;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }

  @Override
  public void
      simulationPeriodic() { // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Final Setpoint", () -> m_setpoint, null);
    builder.addDoubleProperty("Position", () -> m_encoder.getPosition(), null);
    builder.addDoubleProperty("Applied Output", () -> m_motor.getAppliedOutput(), null);
    builder.addDoubleProperty("Elapsed Time", () -> m_timer.get(), null);
    /*builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);*/
    builder.addDoubleProperty("Feedforward", () -> m_feedforward, null);
    builder.addDoubleProperty("Manual Value", () -> m_manualValue, null);
    // builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    // builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    // addChild("Controller", m_controller);
  }
}
