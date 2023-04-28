// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX rollers;
  private final TalonFX arm;
  private PIDController pidController;
  private double desiredAngle;
  private IntakeModes mode;  
  private ShuffleboardTab intakeShuffleboard = Shuffleboard.getTab("Intake");


//constructor
  public IntakeSubsystem() {
    rollers = new TalonFX(Constants.GroundIntake.ROLLERS_MOTOR);
    arm = new TalonFX(Constants.GroundIntake.ARM_MOTOR);
    //current mode of subsytem
    IntakeModes mode = IntakeModes.OFF;
    pidController = new PIDController(0.005, 0, 0.00017);


    intakeShuffleboard.addNumber("Arm Angle", arm::getSelectedSensorPosition);
  }

  public enum IntakeModes{
    INTAKE,
    OFF
  }

  public double getAngle(){
    return arm.getSelectedSensorPosition();
  }

  public double getTargetAngle(){
    return desiredAngle;
  }

  public void setAngle(double desiredAngle){
    this.desiredAngle = desiredAngle;
  }

  //get current mode
  public IntakeModes getMode(){
    return mode;
  }

  //sets the current mode
  public void setMode(IntakeModes mode){
    this.mode = mode;
  }

  public void intakePeriodic(){
    rollers.set(TalonFXControlMode.Velocity, Constants.GroundIntake.ROLLERS_POWER);
    
  }

  public void offPeriodic(){
    rollers.set(TalonFXControlMode.Velocity, Constants.GroundIntake.OFF);

  }
 
  @Override
  public void periodic() {
  
    
    // This method will be called once per scheduler run
  }
}
