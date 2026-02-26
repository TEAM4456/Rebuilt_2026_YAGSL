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

public class Intake extends SubsystemBase{

  private SparkMax escalatorMotor;
  private RelativeEncoder escalatorEncoder;
  private SparkClosedLoopController escalatorLoop;
  private SparkMaxConfig escalatorConfig;

  private SparkMax feedMotor;
  private RelativeEncoder feedEncoder;
  private SparkClosedLoopController feedLoop;
  private SparkMaxConfig feedConfig;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    //FIXME put in actual CAN IDs
    //escalatorMotor = new SparkMax(, MotorType.kBrushless);
    escalatorEncoder = escalatorMotor.getEncoder();
    escalatorLoop = escalatorMotor.getClosedLoopController();
    escalatorConfig = new SparkMaxConfig();

    //feedMotor = new SparkMax(, MotorType.kBrushless);
    feedEncoder = feedMotor.getEncoder();
    feedLoop = feedMotor.getClosedLoopController();
    feedConfig = new SparkMaxConfig();
    
  }

  /** Brings the intake mechanism down onto the field @return this command */
  public Command intakeDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
      () -> {
        escalatorLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
      });
  }

  /** Raises intake mechanism off the field @return this command */
  public Command intakeUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
      () -> {
        escalatorLoop.setSetpoint(0, SparkBase.ControlType.kPosition);
      });
  }

  /** Accelerates intake feed motor to its max speed (Still being determined) @return this command */
  public Command intakeStart() {
    return run(
      () -> {
        feedMotor.set(0);
      });
  }

  /** Stops the feed motor @return this command */
  public Command intakeStop() {
    return run(
      () -> {
        feedMotor.set(0); // Leave as zero because we are stopping motors
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
    SmartDashboard.putNumber("intakeSpeed", feedEncoder.getVelocity());
  }
  /*
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  */
}
