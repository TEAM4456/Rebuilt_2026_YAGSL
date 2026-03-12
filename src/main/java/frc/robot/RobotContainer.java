// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// PS: Remy was here, you prob wont find this line of code until 2050 >:)

package frc.robot;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
/* Most commands will be imported later when we deem them necessary...
  import frc.robot.commands.Autos;
  import frc.robot.commands.ExampleCommand;
*/
// ...This command however has proven itself and has been deemed worthy
import frc.robot.commands.TeleopSwerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController second = new CommandXboxController(1);
  private final CommandXboxController backup = new CommandXboxController(2);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value ;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // The robot's subsystems and commands are defined here...

  private final Intake intakeSubsystem = new Intake();
  private final Shooter shooterSubsystem = new Shooter();

  private final SwerveDrive swerve = new SwerveDrive();
  
  private RobotConfig config;

  private final SendableChooser<Command> chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> driver.getRawAxis(translationAxis), // Used to have negatives in front
            () -> driver.getRawAxis(strafeAxis), // This too
            () -> driver.getRawAxis(rotationAxis)/2));


    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto", chooser);
    
    // Configure the trigger bindings
    configureBindings();
  }

  // /** Starts the shooter, waits 1 second, then starts the feeder. Execute only OnTrue */
  public Command shooterShootCommand() {
    return new SequentialCommandGroup(

      shooterSubsystem.shooterShoot(),
      new WaitCommand(1.0),
      shooterSubsystem.feedStart()
    );
  }

  // /** Stops the shooter and feeder simultaniously */
  public Command shooterStopCommand() {
    return new ParallelCommandGroup(

      shooterSubsystem.shooterStop(),
      shooterSubsystem.feedStop()
    );
  }

  // /** If intake is down, raise it. If intake is up, lower it */
  public Command intakeToggleCommand() {

    if (intakeSubsystem.getIsDown())
      return intakeSubsystem.intakePivotTurnUp();
    else
      return intakeSubsystem.intakePivotTurnDown();
  }

  // /** Start intake feed motor */
  public Command intakeStartCommand() {

    return intakeSubsystem.spinStart();
  }

  // /** Stop intake feed motor */
  public Command intakeStopCommand() {

    return intakeSubsystem.spinStop();
  }


  // Calls the SwerveDrive "updateOdometry" method. Then this method is called in "Robot.java" for actual use. This is therefore a bridge method
  public void updateOdometry() {
    swerve.updateOdometry();
  }

  // =======================================================================
  // ========================= AUTONOMOUS COMMANDS =========================
  // =======================================================================

  public Command TestAutoAutoCommand() {
    
    return new PathPlannerAuto("Test Auto");
  }
  
  
  private void configureBindings() {

    chooser.setDefaultOption("nothing", null);

    chooser.addOption("A autonomous routine to test the auto functionality", TestAutoAutoCommand());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Maybe use whileTrue(), idk?
    driver.rightTrigger().onTrue(shooterShootCommand());
    driver.rightTrigger().onFalse(shooterStopCommand());

    driver.leftBumper().onTrue(intakeToggleCommand());

    driver.leftTrigger().whileTrue(intakeStartCommand());
    driver.leftTrigger().whileFalse(intakeStopCommand());
  }
  
  // Look into this later
  /** Takes the selected Auto path in Smart Dashbooard and runs it when "autonomousInit" in "Robot.java" is first started 
   *  @return The command to run in autonomous */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
