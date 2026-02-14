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

public class Shooter extends SubsystemBase{

  private SparkMax shootLeftMotor;
  private RelativeEncoder shootLeftEncoder;
  private SparkClosedLoopController shootLeftLoop;
  private SparkMaxConfig shootLeftConfig;

  private SparkMax shootRightMotor;
  private RelativeEncoder shootRightEncoder;
  private SparkClosedLoopController shootRightLoop;
  private SparkMaxConfig shootRightConfig;

  private SparkMax feedMotor;
  private RelativeEncoder feedEncoder;
  private SparkClosedLoopController feedLoop;
  private SparkMaxConfig feedConfig;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    shootLeftMotor = new SparkMax(19, MotorType.kBrushless);
    shootLeftEncoder = shootLeftMotor.getEncoder();
    shootLeftLoop = shootLeftMotor.getClosedLoopController();
    shootLeftConfig = new SparkMaxConfig();

    shootRightMotor = new SparkMax(18, MotorType.kBrushless);
    shootRightEncoder = shootRightMotor.getEncoder();
    shootRightLoop = shootRightMotor.getClosedLoopController();
    shootRightConfig = new SparkMaxConfig();

    feedMotor = new SparkMax(17, MotorType.kBrushless);
    feedEncoder = feedMotor.getEncoder();
    feedLoop = feedMotor.getClosedLoopController();
    feedConfig = new SparkMaxConfig();

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ShooterShoot() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          shootLeftLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
          shootRightLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
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
