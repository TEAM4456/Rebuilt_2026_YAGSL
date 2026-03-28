// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

public class Shooter extends SubsystemBase{

  private SparkMax shootLeftMotor; // Represents
  private RelativeEncoder shootLeftEncoder;
  private SparkClosedLoopController shootLeftLoop;
  private SparkMaxConfig shootLeftConfig;

  private SparkMax shootRightMotor;
  private RelativeEncoder shootRightEncoder;
  private SparkClosedLoopController shootRightLoop;
  private SparkMaxConfig shootRightConfig;

  public Shooter() {

    shootLeftMotor = new SparkMax(19, MotorType.kBrushless);
    shootLeftEncoder = shootLeftMotor.getEncoder();
    shootLeftLoop = shootLeftMotor.getClosedLoopController();

    shootLeftConfig = new SparkMaxConfig();
    shootLeftConfig.idleMode(IdleMode.kCoast);
    shootLeftConfig.closedLoop
    .p(0.0005, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
    .feedForward
    // kV is now in Volts, so we multiply by the nominal voltage (12V)
    .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    shootLeftConfig.closedLoop
    .p(0.0005, ClosedLoopSlot.kSlot2)
    .i(0, ClosedLoopSlot.kSlot2)
    .d(0, ClosedLoopSlot.kSlot2)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot2)
    .feedForward
    // kV is now in Volts, so we multiply by the nominal voltage (12V)
    .kV(12.0 / 5767, ClosedLoopSlot.kSlot2);

    shootLeftConfig.closedLoop.maxMotion.
      cruiseVelocity(4600).
      maxAcceleration(5000).
      allowedProfileError(1);
    shootLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shootLeftConfig.openLoopRampRate(0.5);
    shootLeftConfig.smartCurrentLimit(40);
    shootLeftMotor.configure(shootLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    shootRightMotor = new SparkMax(18, MotorType.kBrushless);
    shootRightEncoder = shootRightMotor.getEncoder();
    shootRightLoop = shootRightMotor.getClosedLoopController();

    shootRightConfig = new SparkMaxConfig();
    shootRightConfig.idleMode(IdleMode.kCoast);

    shootRightConfig.closedLoop
      .p(0.0005, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
      .feedForward
      // kV is now in Volts, so we multiply by the nominal voltage (12V)
      .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

      shootRightConfig.closedLoop
      .p(0.0005, ClosedLoopSlot.kSlot2)
      .i(0, ClosedLoopSlot.kSlot2)
      .d(0, ClosedLoopSlot.kSlot2)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot2)
      .feedForward
      // kV is now in Volts, so we multiply by the nominal voltage (12V)
      .kV(12.0 / 5767, ClosedLoopSlot.kSlot2);

    shootRightConfig.closedLoop.maxMotion.
      cruiseVelocity(4800).
      maxAcceleration(5000).
      allowedProfileError(1);
    shootRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shootRightConfig.openLoopRampRate(0.5);
    shootRightConfig.smartCurrentLimit(40);
    shootRightMotor.configure(shootRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Starts both shooters spinning in shoot direction @return this command */
  public Command shooterShootTrench() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftLoop.setSetpoint(3350, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        shootRightLoop.setSetpoint(-3350, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      });
  }
  public Command shooterShootPassing() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftLoop.setSetpoint(4100, ControlType.kVelocity, ClosedLoopSlot.kSlot2);
        shootRightLoop.setSetpoint(-4100, ControlType.kVelocity, ClosedLoopSlot.kSlot2);
      });
    }
   public Command shooterShootAgainstHub() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftLoop.setSetpoint(2550, ControlType.kVelocity, ClosedLoopSlot.kSlot2);
        shootRightLoop.setSetpoint(-2550, ControlType.kVelocity, ClosedLoopSlot.kSlot2);
      });
  }

  /** Stops both shooters and index motor basically shutting down the shooter @return this command */
  public Command shooterStop() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0);
        shootRightMotor.set(0);
      });
  }

  public Command shooterReverse() {
    return run(
      () -> {

        // Sets these motors to the reverse
        shootLeftMotor.set(-0.3);
        shootRightMotor.set(0.3);
    });
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Shooter Speed", shootRightEncoder.getVelocity());
    SmartDashboard.putNumber("Right Motor Position", shootRightEncoder.getPosition());

    SmartDashboard.putNumber("Left Shooter Speed", shootLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Left Motor Position", shootLeftEncoder.getPosition());
  }
}