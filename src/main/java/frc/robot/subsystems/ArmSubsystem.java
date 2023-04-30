package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {
  private double desiredAngle;
  private TalonFX telescopingMotor;
  private PIDController pidController;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Arm");
  private ArmModes mode;
  private LinearFilter lowPassFilter;
  private double filterOutput;

  public enum ArmModes {
    MANUAL,
    EXTEND,
    RETRACT,
    HOLD
  }

  public static record ArmState(double angle, double extension) {}

  public ArmSubsystem() {
    telescopingMotor = new TalonFX(Constants.Arm.Ports.TELESCOPING_MOTOR_PORT);
    pidController = new PIDController(0, 0, 0);
    lowPassFilter = LinearFilter.movingAverage(5);
    shuffleboard.add("PID", pidController);
    shuffleboard.addNumber("Low Pass Filter", () -> updateFilterOutput());
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public void PIDPeriodic(double desiredAngle) {
    telescopingMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            pidController.calculate(telescopingMotor.getSelectedSensorPosition(), desiredAngle),
            -0.25,
            0.25));
  }

  public void setMode(ArmModes mode) {
    this.mode = mode;
  }

  public double updateFilterOutput() {
    return lowPassFilter.calculate(telescopingMotor.getStatorCurrent());
  }

  private static double ticksLengthToInches(double ticks) {
    return ((ticks / Arm.TICKS) / Arm.TELESCOPING_ARM_GEAR_RATIO) * Arm.SPOOL_CIRCUMFERENCE;
  }

  public double getCurrentExtensionInches() {
    return ticksLengthToInches(telescopingMotor.getSelectedSensorPosition());
  }

  public void applyMode() {
    switch (mode) {
      case EXTEND:
        PIDPeriodic(0);
      case RETRACT:
        PIDPeriodic(0);
      case HOLD:
        telescopingMotor.setNeutralMode(NeutralMode.Brake);
      default:
        telescopingMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public ArmModes advanceMode(ArmModes currentMode) {
    if (filterOutput >= Constants.Arm.EXTENSION_STATOR_LIMIT
        || getCurrentExtensionInches() == Constants.Arm.Setpoints.GROUND_INTAKE.extension
        || getCurrentExtensionInches() == Constants.Arm.Setpoints.SHELF_INTAKE.extension) {
      return ArmModes.HOLD;
    } else if (currentMode == ArmModes.EXTEND) {
      return ArmModes.EXTEND;
    } else if (currentMode == ArmModes.RETRACT) {
      return ArmModes.RETRACT;
    } else {
      return ArmModes.HOLD;
    }
  }

  public void periodic() {
    mode = advanceMode(mode);
    applyMode();
    filterOutput = updateFilterOutput();
  }
}
