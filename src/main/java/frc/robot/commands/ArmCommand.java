package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmModes;

public class ArmCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private double desiredAngle;
  private double extendedAngle;
  private double retractedAngle;

  public ArmCommand(ArmSubsystem armSubsystem, double desiredAngle) {
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
    extendedAngle = 0;
    retractedAngle = 0;
  }

  public void initialize() {
    armSubsystem.setMode(ArmModes.PID);
  }

  public void extendArm() {
    armSubsystem.setMode(ArmModes.PID);
    armSubsystem.setDesiredAngle(extendedAngle);
  }

  public void execute() {}

  public void end() {}

  public boolean isFinished() {
    return false;
  }
}
