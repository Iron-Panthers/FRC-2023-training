package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private double desiredAngle;
  private TalonFX telescopingMotor;
  private PIDController pidController;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Arm");
  private Modes mode;

  public enum Modes {
    PID,
    MANUAL,
    HOLD
  }

  public ArmSubsystem() {
    telescopingMotor = new TalonFX(Constants.Arm.Ports.TELESCOPING_MOTOR_PORT);
    pidController = new PIDController(0, 0, 0);

    shuffleboard.add("PID", pidController);
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public void PIDPeriodic() {
    telescopingMotor.set(
        TalonFXControlMode.PercentOutput,
        MathUtil.clamp(
            pidController.calculate(telescopingMotor.getSelectedSensorPosition(), desiredAngle),
            -0.1,
            0.1));
  }

  public void manualPeriodic(double speed) {
    telescopingMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  public void applyMode() {
    switch (mode) {
      case PID:
        PIDPeriodic();
      case MANUAL:
        manualPeriodic(0);
    }
  }

  public void periodic() {}
}
