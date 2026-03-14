// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

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

  private SparkMax indexMotor;
  private RelativeEncoder indexEncoder;
  private SparkClosedLoopController indexLoop;
  private SparkMaxConfig indexConfig;

  public Shooter() {
    shootLeftMotor = new SparkMax(19, MotorType.kBrushless);
    shootLeftEncoder = shootLeftMotor.getEncoder();
    shootLeftLoop = shootLeftMotor.getClosedLoopController();

    shootLeftConfig = new SparkMaxConfig();
    shootLeftConfig.idleMode(IdleMode.kCoast);
    shootLeftConfig.closedLoop.pid(1, 0, 0);
    shootLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shootLeftConfig.openLoopRampRate(0.5);
    shootLeftConfig.smartCurrentLimit(40);
    shootLeftMotor.configure(shootLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    shootRightMotor = new SparkMax(18, MotorType.kBrushless);
    shootRightEncoder = shootRightMotor.getEncoder();
    shootRightLoop = shootRightMotor.getClosedLoopController();

    shootRightConfig = new SparkMaxConfig();
    shootRightConfig.idleMode(IdleMode.kCoast);
    shootRightConfig.closedLoop.pid(1, 0, 0);
    shootRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shootRightConfig.openLoopRampRate(0.5);
    shootRightConfig.smartCurrentLimit(40);
    shootRightMotor.configure(shootRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    
    indexMotor = new SparkMax(17, MotorType.kBrushless);
    indexEncoder = indexMotor.getEncoder();
    indexLoop = indexMotor.getClosedLoopController();

    indexConfig = new SparkMaxConfig();
    indexConfig.idleMode(IdleMode.kCoast);
    indexConfig.closedLoop.pid(1, 0, 0);
    indexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    indexConfig.openLoopRampRate(0.5);
    indexConfig.smartCurrentLimit(40);
    indexMotor.configure(indexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Starts both shooters spinning in shoot direction @return this command */
  public Command shooterShoot() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0.2);
        shootRightMotor.set(0.2);
      });
  }

  /** Stops both shooters and index motor basically shutting down the shooter @return this command */
  public Command shooterStop() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0);
        shootRightMotor.set(0);

        indexStop(); // Needed to be called here to avoid a "multiple thread exception" in robot container
      });
  }

  /** Starts the feed motor @return this command */
  public Command indexStart() {
    return run(
      () -> {

        indexMotor.set(0.2);
      });
  }

  /** Use to reverse indexer and unclog jams potentially? @return this command */
  public Command indexReverse() {
    return run(
      () -> {

        indexMotor.set(-0.2);
      });
  }

  /** Stops the index motor @return this command */
  public Command indexStop() {
    return run(
      () -> {

        indexMotor.set(0);
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shootRightMotor", shootRightEncoder.getVelocity());
    SmartDashboard.putNumber("shootLeftMotor", shootLeftEncoder.getVelocity());
  }
}