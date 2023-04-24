package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Modes;

public class ArmCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private double desiredAngle;

  public ArmCommand(ArmSubsystem armSubsystem, double desiredAngle) {
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;
  }

  public void initialize() {
    armSubsystem.setMode(Modes.PID);
  }

  public void extendArm() {
    armSubsystem.setMode(Modes.PID);
    armSubsystem.setDesiredAngle(0);
  }

  public void execute() {}

  public void end() {}

  public boolean isFinished() {
    return false;
  }
}
