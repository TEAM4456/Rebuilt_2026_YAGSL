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

public class ShootFeed extends SubsystemBase{

  private SparkMax feedMotor;
  private RelativeEncoder feedEncoder;
  private SparkClosedLoopController feedLoop;
  private SparkMaxConfig feedConfig;

  private SparkMax indexMotor;
  private RelativeEncoder indexEncoder;
  private SparkClosedLoopController indexLoop;
  private SparkMaxConfig indexConfig;

  public ShootFeed() {
    
    feedMotor = new SparkMax(17, MotorType.kBrushless);
    feedEncoder = feedMotor.getEncoder();
    feedLoop = feedMotor.getClosedLoopController();

    feedConfig = new SparkMaxConfig();
    feedConfig.idleMode(IdleMode.kCoast);
    feedConfig.closedLoop.pid(1, 0, 0);
    feedConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    feedConfig.openLoopRampRate(0.5);
    feedConfig.smartCurrentLimit(40);
    feedMotor.configure(feedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    indexMotor = new SparkMax(16, MotorType.kBrushless);
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

  /** Starts the feed motor @return this command */
  public Command shootFeedStart() {
    return run(
      () -> {

        feedMotor.set(-1);
        indexMotor.set(-0.6);
      });
  }

  /** Use to reverse indexer and unclog jams potentially? @return this command */
  public Command shootFeedReverse() {
    return run(
      () -> {

        feedMotor.set(0.5);
        indexMotor.set(0.3);
      });
  }

  /** Stops the index motor @return this command */
  public Command shootFeedStop() {
    return run(
      () -> {

        feedMotor.set(0);
        indexMotor.set(0);
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Feed Motor Speed", feedEncoder.getVelocity());
    SmartDashboard.putNumber("Feed Motor Position", feedEncoder.getPosition());

    SmartDashboard.putNumber("Index Motor Speed", indexEncoder.getVelocity());
    SmartDashboard.putNumber("Feed Motor Position", indexEncoder.getPosition());

  }
}