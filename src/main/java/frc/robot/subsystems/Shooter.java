// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

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

  /** Starts both shooters spinning in shoot direction @return this command */
  public Command shooterShoot() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0.2);
        shootRightMotor.set(0.2);
      });
  }

  /** Stops both shooters and feed motor basically shutting down the shooter @return this command */
  public Command shooterStop() {
    return run(
      () -> {

        // Set these to the same speed
        shootLeftMotor.set(0);
        shootRightMotor.set(0);

        feedMotor.set(0);
      });
  }

  /** Starts the feed motor @return this command */
  public Command feedStart() {
    return run(
      () -> {

        feedMotor.set(0.2);
      });
  }

  /** Use to reverse feeder and unclog jams potentially? @return this command */
  public Command feederReverse() {
    return run(
      () -> {

        feedMotor.set(-0.2);
      });
  }

  /** Stops the feed motor @return this command */
  public Command feedStop() {
    return run(
      () -> {

        feedMotor.set(0);
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    SmartDashboard.putNumber("shootRightMotor", shootRightEncoder.getVelocity());
    SmartDashboard.putNumber("shootLeftMotor", shootLeftEncoder.getVelocity());
  }
}
