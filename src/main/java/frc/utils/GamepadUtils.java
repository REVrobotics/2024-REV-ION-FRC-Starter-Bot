package frc.utils;

import edu.wpi.first.math.MathUtil;

public class GamepadUtils {
  public static double squareInput(double _value, double _deadband) {
    return Math.pow(MathUtil.applyDeadband(_value, _deadband), 2) * Math.signum(_value);
  }
}
