// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/* Most commands will be imported later when we deem them necessary...
  import frc.robot.commands.Autos;
  import frc.robot.commands.ExampleCommand;
*/
// ...This command however has proven itself and has been deemed worthy
import frc.robot.commands.TeleopSwerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  private final Elevator elevatorSubsystem = new Elevator();
  private final Intake intakeSubsystem = new Intake();
  private final Shooter shooterSubsystem = new Shooter();
  private final SwerveDrive swerve = new SwerveDrive();

  private final SendableChooser<Command> chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis)/2));

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);
    
    // Configure the trigger bindings
    configureBindings();
    

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  }

  // Calls the SwerveDrive "updateOdometry" method. Then this method is called in "Robot.java" for actual use. This is therefore a bridge method
  public void updateOdometry() {
    swerve.updateOdometry();
  }

  /*
  public Command elevatorUpCommand() {
    return elevatorSubsystem.elevatorUp();
  }
  public Command resetElevatorCommand() {
    return elevatorSubsystem.resetElevator();
  }
  */
  
  
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Look into this later

    //new Trigger(elevatorSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(elevatorSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(elevatorSubsystem.exampleMethodCommand());
    
    chooser.setDefaultOption("nothing", null);
    //chooser.addOption("C to 1 R", centerTo1RightAutoCommand());

  
  }
  
  // Look into this later
  /** Takes the selected Auto path in Smart Dashbooard and runs it when "autonomousInit" in "Robot.java" is first started 
   *  @return The command to run in autonomous */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
