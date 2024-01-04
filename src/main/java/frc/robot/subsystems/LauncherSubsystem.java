package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_topMotor;
  private CANSparkMax m_bottomMotor;

  public LauncherSubsystem() {
    m_topMotor =
        new CANSparkMax(Constants.Launcher.kTopCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_topMotor.setInverted(false);
    m_topMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_topMotor.setIdleMode(IdleMode.kBrake);

    m_topMotor.burnFlash();

    m_bottomMotor =
        new CANSparkMax(Constants.Launcher.kBottomCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_bottomMotor.setInverted(false);
    m_bottomMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_bottomMotor.setIdleMode(IdleMode.kBrake);

    m_bottomMotor.burnFlash();
  }

  public void runLauncher() {
    m_topMotor.set(Constants.Launcher.kTopPower);
    m_bottomMotor.set(Constants.Launcher.kBottomPower);
  }

  public void stopLauncher() {
    m_topMotor.set(0.0);
    m_bottomMotor.set(0.0);
  }
}
