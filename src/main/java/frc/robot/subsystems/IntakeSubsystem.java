// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX outake;

  public enum IntakeModes{
    INTAKE,
    HOLD, 
    OUTAKE,
    OFF
  }

  //current mode of subsytem
  private IntakeModes mode = IntakeModes.OFF;

  //hold in mode, keep from transitioning
  private boolean modeLocked = false;

  //get current mode
  public IntakeModes getMode(){
    return mode;
  }
  //gets boolean for mode locked
  public boolean getModeLocked(){
  return modeLocked;
  }

  public double timeSinceModeTransition(){
    return Timer.getFPGATimestamp() - timeOfModeTransition;
  }

  public void setModeIfTransitionGreater(double duration, IntakeModes mode){
    if(timeSinceModeTransition() >= duration){
      setMode(mode);
    }
  }

  //sets the current mode
  public void setMode(IntakeModes mode){
    this.mode = mode;
  }

  public IntakeSubsystem() {
    outake = new TalonFX(Constants.GroundIntake.OUTTAKE_MOTOR);
  }

  public void intakePeriodic(){
    outake.set(TalonFXControlMode.Velocity, Constants.GroundIntake.INTAKE);
  }

  public void holdPeriodic(){
    outake.set(TalonFXControlMode.Velocity, Constants.GroundIntake.HOLD);
  }

  public void outakePeriodic(){
    outake.set(TalonFXControlMode.Velocity, Constants.GroundIntake.OUTTAKE);
  }

  public void offPeriodic(){
    outake.set(TalonFXControlMode.Velocity, Constants.GroundIntake.OFF);
  }

 
  @Override
  public void periodic() {
  
    
    
    // This method will be called once per scheduler run
  }
}
