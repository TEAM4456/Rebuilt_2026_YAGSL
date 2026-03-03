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
//TODO: Change class name to Climber - should only have one motor CanID 15
  private SparkMax innerMotor;
  private RelativeEncoder innerEncoder;
  private SparkClosedLoopController innerLoop;
  private SparkMaxConfig innerConfig;

  private SparkMax outerMotor;
  private RelativeEncoder outerEncoder;
  private SparkClosedLoopController outerLoop;
  private SparkMaxConfig outerConfig;

  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private SparkClosedLoopController climberLoop;
  private SparkMaxConfig climberConfig;

  /** Creates a new ExampleSubsystem. */
  public Climber() {
    innerMotor = new SparkMax(14, MotorType.kBrushless);
    innerEncoder = innerMotor.getEncoder();
    innerLoop = innerMotor.getClosedLoopController();
    innerConfig = new SparkMaxConfig();

    outerMotor = new SparkMax(15, MotorType.kBrushless);
    outerEncoder = outerMotor.getEncoder();
    outerLoop = outerMotor.getClosedLoopController();
    outerConfig = new SparkMaxConfig();
    
    climberMotor = new SparkMax(16, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberLoop = climberMotor.getClosedLoopController();
    climberConfig = new SparkMaxConfig();

    // Get climber position for later reset?
    
  }

  /** Moves the climber mechanism either up to a specific point or continuously at a set speed. We dont know which yet @return this command */
  public Command climberUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
      () -> {
        // Goes to specific point
        climberLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
        // Or sets speed and continuouly runs
        climberMotor.set(0.2);
      });
  }

  /** Sets the inner and outer claws on the mechanism to their "Bar grab point" @return this command */
  public Command clawHook() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // Run both as set point
          innerLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
          outerLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
        });
  }

  /** Resets the climber mechanism and claws to their starting positions @return this command */
  public Command resetClimber() {
    return runOnce(
      () -> {
          // Run all as setpoints
          climberLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
          innerLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
          outerLoop.setSetpoint(0, SparkBase.ControlType.kPosition);

        });
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
