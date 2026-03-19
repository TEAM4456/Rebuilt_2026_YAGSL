// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// PS: Remy was here, you prob wont find this line of code until 2050 >:)

package frc.robot;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShootFeed;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ShootFeed shootFeedSubsystem = new ShootFeed();

  private final SwerveDrive swerve = new SwerveDrive();
  
  private RobotConfig config;

  private final SendableChooser<Command> chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // If we are red alliance, then the default Field view is good. Otherwise, we need
    //  to invert the translation and strafe axes, as well as the drive motors in SwerveDrive
    boolean isRed = swerve.allianceIsRed();
    int invertAxes = isRed ? 1 : -1;
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> driver.getRawAxis(translationAxis) * invertAxes,
            () -> driver.getRawAxis(strafeAxis) * invertAxes,
            () -> driver.getRawAxis(rotationAxis) / 2));

    shooterSubsystem.setDefaultCommand(shooterStopCommand());
    intakeSubsystem.setDefaultCommand(intakeStopCommand());


    chooser = AutoBuilder.buildAutoChooser();
    
    // Configure the trigger bindings
    configureBindings();
    
    SmartDashboard.putData("auto", chooser);
  }

  /** Starts the shooter for Trench shot */
  public Command shooterShootTrenchCommand() {
    return shooterSubsystem.shooterShootTrench();
  }

  /** Starts the shooter for close shot up against Hub */
  public Command shooterShootUpAgainstHubCommand() {
    return shooterSubsystem.shooterShootAgainstHub();
  }

  /** Starts the feeder motors (feed and indexer) */
  public Command ShootFeedCommand() {
    return shootFeedSubsystem.shootFeedStart();
  }

   /** Reverses the feeder motors (feed and indexer) */
  public Command ShootFeedReverseCommand() {
    return shootFeedSubsystem.shootFeedReverse();
  }

  // /** Stops the shooter */
  public Command shooterStopCommand() {
    return shooterSubsystem.shooterStop();
  }

  /* Stops the feeder */
   public Command feederStopCommand() {
    return shootFeedSubsystem.shootFeedStop();
  }

  // /** If intake is down, raise it. If intake is up, lower it */
  public Command intakeToggleCommand() {

    if (intakeSubsystem.getIsDown())
      return intakeSubsystem.intakePivotTurnUp();
    else
      return intakeSubsystem.intakePivotTurnDown();
  }

  public Command intakeUpCommand() {
    return intakeSubsystem.intakePivotTurnUp();
  }

  public Command intakeDownCommand() {
    return intakeSubsystem.intakePivotTurnDown();
  }

  // /** Start intake feed motor */
  public Command intakeStartCommand() {

    return intakeSubsystem.spinStart();
  }

  // /** Reverses intake feed motor */
  public Command intakeReverseCommand() {

    return intakeSubsystem.spinReverse();
  }

  // /** Stop intake feed motor */
  public Command intakeStopCommand() {

    return intakeSubsystem.spinStop();
  }
  public Command stopAllMotars() {
    return new ParallelCommandGroup(intakeStopCommand(), feederStopCommand(), shooterStopCommand());
  }

  // Calls the SwerveDrive "updateOdometry" method. Then this method is called in "Robot.java" for actual use. This is therefore a bridge method
  public void updateOdometry() {
    swerve.updateOdometry();
  }

  // =======================================================================
  // ========================= AUTONOMOUS COMMANDS =========================
  // =======================================================================

  public Command forwardAutoCommand(){

    return new PathPlannerAuto("Forward Auto");
  }

  public Command spinningRobotShootAutoCommand() {

    return new SequentialCommandGroup(

      new PathPlannerAuto("Spinning Robot Auto"),
      intakeStartCommand().withTimeout(3),
      intakeStopCommand()
    );
  }
  
  
  private void configureBindings() {

    chooser.setDefaultOption("nothing", null);
    chooser.addOption("Move Forward", forwardAutoCommand());
    chooser.addOption("Spin and Shoot", spinningRobotShootAutoCommand());
    

    // Maybe use whileTrue(), idk?
    driver.rightTrigger().toggleOnTrue(shooterShootTrenchCommand());

    driver.rightBumper().toggleOnTrue(shooterShootUpAgainstHubCommand());
    

    driver.leftTrigger().whileTrue(ShootFeedCommand());
    driver.leftTrigger().whileFalse(feederStopCommand());

    driver.leftBumper().whileTrue(ShootFeedReverseCommand());
    driver.leftBumper().whileFalse(feederStopCommand());

    //driver.leftBumper().onTrue(intakeToggleCommand());

    driver.a().onTrue(intakeDownCommand());
    driver.y().onTrue(intakeUpCommand());

    driver.b().toggleOnTrue(intakeStartCommand());
    driver.x().whileTrue(intakeReverseCommand());
    //driver.back().whileTrue(autoAlignShootBlueLeftCommand());
    driver.povDown().and(driver.povUp().negate()).and(driver.povLeft().negate()).and(driver.povRight()).onTrue(stopAllMotars());
  }
  
  // Look into this later
  /** Takes the selected Auto path in Smart Dashbooard and runs it when "autonomousInit" in "Robot.java" is first started 
   *  @return The command to run in autonomous */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
