package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmModes;
import org.apache.commons.math3.analysis.function.Constant;

public class ArmCommand extends CommandBase {
  private ArmSubsystem armSubsystem;

  public ArmCommand(ArmSubsystem armSubsystem, double desiredAngle) {
    this.armSubsystem = armSubsystem;
  }

  public void initialize() {
    armSubsystem.setMode(ArmModes.HOLD);
  }

  public void extendArm() {
    armSubsystem.setMode(ArmModes.EXTEND);
    armSubsystem.setDesiredAngle(Constant.Arm.Setpoints.SHELF_INTAKE.extension);
  }

  public void retractArm() {
    armSubsystem.setMode(ArmModes.RETRACT);
    armSubsystem.setDesiredAngle(Constant.Arm.Setpoints.GROUND_INTAKE.extension);
  }

  public void execute() {}

  public void end() {}

  public boolean isFinished() {
    return false;
  }
}
