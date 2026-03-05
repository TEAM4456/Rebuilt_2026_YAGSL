// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase{


  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private SparkClosedLoopController climberLoop;
  private SparkMaxConfig climberConfig;

  /** Creates a new ExampleSubsystem. */
  public Climber() {
    
    climberMotor = new SparkMax(15, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberLoop = climberMotor.getClosedLoopController();
    climberConfig = new SparkMaxConfig();

    // Get climber position for later reset?
    
  }

  // Manual controls

  /** Moves the climber mechanism at a set speed. Choose this if you want set speed @return this command */
  public Command climberTurnUp() { 
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
      () -> {
        // Goes to specific point
        // climberLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
        // Or sets speed and continuouly runs
        climberMotor.set(0.2);
      });
  }

  /** Moves the climber mechanism at a set speed down. Choose this if you want continuous speed @return this command */
  public Command climberTurnDown() { 
    return run(
      () -> {
        climberMotor.set(-0.2);
      });
  }

  // Automatic controls

  /** Moves the climber mechanism up to a specific point. Choose this if at a set position @return this command */
  public Command climberUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
      () -> {
        // Goes to specific point
        // TODO: Find setpoint for climbing up
        climberLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
        // Or sets speed and continuouly runs
        // climberMotor.set(0.2);
      });
  }

  /** Moves the climber mechanism down to a specific point. Choose this if at a set position @return this command */
  public Command climberDown() { //TODO: Find out of this is necessary or if resetClimber is enough for going down
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
      () -> {
        climberLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
      });
  }

  /** Resets the climber pivot to its starting positions @return this command */
  public Command resetClimber() {
    return runOnce(
      () -> {

        climberLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climberPosition", climberEncoder.getPosition());
  }
  /*
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  */
}
