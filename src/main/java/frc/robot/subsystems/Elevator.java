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

public class Elevator extends SubsystemBase{

  private SparkMax innerMotor;
  private RelativeEncoder innerEncoder;
  private SparkClosedLoopController innerLoop;
  private SparkMaxConfig innerConfig;

  private SparkMax outerMotor;
  private RelativeEncoder outerEncoder;
  private SparkClosedLoopController outerLoop;
  private SparkMaxConfig outerConfig;

  private SparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private SparkClosedLoopController elevatorLoop;
  private SparkMaxConfig elevatorConfig;

  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    innerMotor = new SparkMax(14, MotorType.kBrushless);
    innerEncoder = innerMotor.getEncoder();
    innerLoop = innerMotor.getClosedLoopController();
    innerConfig = new SparkMaxConfig();

    outerMotor = new SparkMax(15, MotorType.kBrushless);
    outerEncoder = outerMotor.getEncoder();
    outerLoop = outerMotor.getClosedLoopController();
    outerConfig = new SparkMaxConfig();
    
    elevatorMotor = new SparkMax(16, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorLoop = elevatorMotor.getClosedLoopController();
    elevatorConfig = new SparkMaxConfig();

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command elevatorUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          elevatorLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
        });
  }

  public Command elevatorDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          elevatorLoop.setSetpoint(-0, SparkBase.ControlType.kPosition);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
