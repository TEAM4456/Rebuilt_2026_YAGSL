// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// PS: Remy was here, you prob wont find this line of code until 2050 >:)

package frc.robot;

// Our own imports
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShootFeed;
import frc.robot.commands.TeleopSwerve;

// External imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

  private final CommandXboxController driver = new CommandXboxController(0); // Represents main driver's xbox controller
  private final CommandXboxController second = new CommandXboxController(1); // Represents secondary xbox controller
  private final CommandXboxController backup = new CommandXboxController(2); // Represents backup xbox controller

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
    // boolean isRed = swerve.allianceIsRed();
    // int invertAxes = isRed ? 1 : -1;
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis) / 2));

    shooterSubsystem.setDefaultCommand(shooterStopCommand());
    intakeSubsystem.setDefaultCommand(intakeStopCommand());


    chooser = AutoBuilder.buildAutoChooser();
    
    // Configure the trigger bindings
    configureBindings();
    swerve.resetPose(null);
    
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

  public Command shooterShootPassingCommand() {
    return shooterSubsystem.shooterShootPassing();
  }

  /** Starts the feeder motors (feed and indexer) */
  public Command shootFeedCommand() {
    return shootFeedSubsystem.shootFeedStart();
  }

   /** Reverses the feeder motors (feed and indexer) */
  public Command shootFeedReverseCommand() {
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

  /** Reverses both the shoot feed mechanism and the actual shooter moters */
  public Command shooterAndShootFeedReverseCommand() {
    return new ParallelCommandGroup(
      shooterSubsystem.shooterReverse(),
      shootFeedSubsystem.shootFeedReverse()
    );
  }

  public Command shooterAndShootFeedStopCommand() {
    return new ParallelCommandGroup(
      shooterSubsystem.shooterStop(),
      shootFeedSubsystem.shootFeedStop()
    );
  }

  /** If intake is down, raise it. If intake is up, lower it */
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

  public Command intakeAngleUpCommand() {
    return intakeSubsystem.intakePivotAngleUp();
  }

  public Command intakeAngleDownCommand() {
    return intakeSubsystem.intakePivotAngleDown();
  }

  public Command intakeAngleStopCommand() {
    return intakeSubsystem.intakePivotAngleStop();
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
  public Command stopAllMotors() {
    return new ParallelCommandGroup(intakeStopCommand(), feederStopCommand(), shooterStopCommand());
  }

  // =======================================================================
  // ========================= AUTONOMOUS COMMANDS =========================
  // =======================================================================

  public Command forwardAutoCommand(){

    return new SequentialCommandGroup(

      intakeDownCommand(),
      new PathPlannerAuto("Forward Auto")
    );
  }

  public Command spinningRobotShootAutoCommand() {

    return new SequentialCommandGroup(

      new PathPlannerAuto("Spinning Robot Auto"),
      intakeStartCommand().withTimeout(3),
      intakeStopCommand()
    );
  }
  public Command blue1MidPickupAndShootCommand() {
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        blue1MidPickupAutoCommand()

        
      ),
      new SequentialCommandGroup(
        new WaitCommand(12),
        shooterShootUpAgainstHubCommand().withTimeout(6)
      ),
      new SequentialCommandGroup(
        new WaitCommand(14),
        shootFeedCommand().withTimeout(5)
      )
    );
  }
  public Command blue1MidPickupAutoCommand() {
    return new SequentialCommandGroup(

      intakeDownCommand().withTimeout(1),
      new ParallelCommandGroup(
        intakeStartCommand().withTimeout(12),
        new PathPlannerAuto("Blue 1 Mid Pickup Auto")
      ),
      intakeStopCommand().withTimeout(1),
      new WaitCommand(4),
      intakeUpCommand()
    );
  }

  public Command red1MidPickupAndShootCommand() {
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        red1MidPickupAutoCommand()
      ),
      new SequentialCommandGroup(
        new WaitCommand(12),
        shooterShootUpAgainstHubCommand().withTimeout(6)
      ),
      new SequentialCommandGroup(
        new WaitCommand(14),
        shootFeedCommand().withTimeout(5)
      )
    );
  }

  public Command red1MidPickupAutoCommand() {
    return new SequentialCommandGroup(

      intakeDownCommand().withTimeout(1),
      new ParallelCommandGroup(
        intakeStartCommand().withTimeout(12),
        new PathPlannerAuto("Red 1 Mid Pickup Auto")
      ),
      intakeStopCommand().withTimeout(1),
      new WaitCommand(4),
      intakeUpCommand()
    );
  }

  public Command blue2ShootPreloadAutoCommand() {
    return new SequentialCommandGroup(

      new PathPlannerAuto("Blue 2 Shoot Preload Auto"),
      intakeDownCommand(),
      new ParallelCommandGroup(
        shooterShootUpAgainstHubCommand().withTimeout(7),
        new SequentialCommandGroup(
          new WaitCommand(2),
          shootFeedCommand().withTimeout(5)
        ),
        new SequentialCommandGroup(
          new WaitCommand(4),
          intakeUpCommand()
        )
      )
    );
  }

  public Command blue3MidPickupAutoCommand() {
    return new SequentialCommandGroup(

      intakeDownCommand(),
      new ParallelCommandGroup(
        intakeStartCommand(),
        new PathPlannerAuto("Blue 3 Mid Pickup Auto").withTimeout(8.0)
      ),
      intakeStopCommand(),
      new ParallelCommandGroup(
        shooterShootTrenchCommand().withTimeout(7),
        new SequentialCommandGroup(
          new WaitCommand(2),
          shootFeedCommand().withTimeout(5)
        ),
        new SequentialCommandGroup(
          new WaitCommand(4),
          intakeUpCommand()
        )
      )
    );
  }
  
  public Command blueXShootInPlaceAutoCommand() {

    return new SequentialCommandGroup(

      intakeDownCommand().withTimeout(2),

      new ParallelCommandGroup(
        shooterShootUpAgainstHubCommand().withTimeout(20),
        new SequentialCommandGroup(
          new WaitCommand(2),
          shootFeedCommand().withTimeout(7)
        ),
        new SequentialCommandGroup(
          new WaitCommand(4),
          intakeUpCommand()
        )
      )
    );
  }

  public Command blueXShootInPlaceAndEXTENDintoMid() {

    return new SequentialCommandGroup(

      intakeDownCommand().withTimeout(2),

      new ParallelCommandGroup(

        shooterShootUpAgainstHubCommand().withTimeout(10),

        new SequentialCommandGroup(
          new WaitCommand(2),
          shootFeedCommand().withTimeout(8)
        )
      ),

      new ParallelCommandGroup(

        intakeStartCommand(),
        new PathPlannerAuto("From Preload Shoot Go Mid Right Auto")
      )
    );
  }
  
  private void configureBindings() {

    chooser.setDefaultOption("nothing", null);
    chooser.addOption("Blue 1 Mid Pickup Auto", blue1MidPickupAutoCommand());
    chooser.addOption("Blue 2 Shoot Preload Auto", blue2ShootPreloadAutoCommand());
    chooser.addOption("Blue 3 Mid Pickup Auto", blue3MidPickupAutoCommand());
    chooser.addOption("Blue Shoot in Place Auto", blueXShootInPlaceAutoCommand());
    chooser.addOption("Shoot Preload, Then Go Mid", blueXShootInPlaceAndEXTENDintoMid());
    chooser.addOption("Blue 1 Collect from Mid, Shoot from Trench", blue1MidPickupAndShootCommand());
    chooser.addOption("Red 1 Collect from Mid, Shoot from Trench", red1MidPickupAndShootCommand());
    chooser.addOption("Move Forward", forwardAutoCommand());
    chooser.addOption("Spin and Shoot", spinningRobotShootAutoCommand());

    // Maybe use whileTrue(), idk?
    driver.rightTrigger().toggleOnTrue(shooterShootTrenchCommand());

    driver.rightBumper().toggleOnTrue(shooterShootUpAgainstHubCommand());
    

    driver.leftTrigger().whileTrue(shootFeedCommand());
    driver.leftTrigger().whileFalse(feederStopCommand());

    driver.leftBumper().whileTrue(shootFeedReverseCommand());
    driver.leftBumper().whileFalse(feederStopCommand());

    //driver.leftBumper().onTrue(intakeToggleCommand());

    driver.a().onTrue(intakeDownCommand());
    driver.y().onTrue(intakeUpCommand());

    driver.b().toggleOnTrue(intakeStartCommand());
    driver.x().whileTrue(intakeReverseCommand());
    //driver.back().whileTrue(autoAlignShootBlueLeftCommand());
    driver.povDown().and(driver.povUp().negate()).and(driver.povLeft().negate()).and(driver.povRight().negate()).onTrue(stopAllMotors());
    driver.povUp().and(driver.povDown().negate()).and(driver.povLeft().negate()).and(driver.povRight().negate()).toggleOnTrue((shooterShootPassingCommand()));

    //Second controller 
    second.rightTrigger().whileTrue(intakeAngleUpCommand());
    second.rightTrigger().whileFalse(intakeAngleStopCommand());

    second.leftTrigger().whileTrue(intakeAngleDownCommand());
    second.rightTrigger().whileFalse(intakeAngleStopCommand());

    second.b().whileTrue(intakeReverseCommand());
    second.b().onFalse(intakeStopCommand());

    second.x().whileTrue(shooterAndShootFeedReverseCommand());
    second.x().onFalse(shooterAndShootFeedStopCommand());

  }

  // Calls the SwerveDrive "updateOdometry" method. Then this method is called in "Robot.java" for actual use. This is therefore a bridge method
  public void updateOdometry() {
    swerve.updateOdometry();
  }

  
  // Look into this later
  /** Takes the selected Auto path in Smart Dashbooard and runs it when "autonomousInit" in "Robot.java" is first started 
   *  @return The command to run in autonomous */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
