// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{

  //Pilot Variables:
  private static Joystick m_Pilot_Controller = new Joystick(0);
  private final JoystickButton m_Pilot_A = new JoystickButton(m_Pilot_Controller, 1);
  private final JoystickButton m_Pilot_B = new JoystickButton(m_Pilot_Controller, 2);
  private final JoystickButton m_Pilot_X = new JoystickButton(m_Pilot_Controller, 3);
  private final JoystickButton m_Pilot_Y = new JoystickButton(m_Pilot_Controller, 4);
  private final JoystickButton m_Pilot_LeftBumper = new JoystickButton(m_Pilot_Controller, 5);
  private final JoystickButton m_Pilot_RightBumper = new JoystickButton(m_Pilot_Controller, 6);
  private final JoystickButton m_Pilot_BACK = new JoystickButton(m_Pilot_Controller, 7);
  private final JoystickButton m_Pilot_START = new JoystickButton(m_Pilot_Controller, 8);
  private final DoubleSupplier m_Pilot_Left_XAxis = () -> (Math.pow(m_Pilot_Controller.getRawAxis(0), 3));
  private final DoubleSupplier m_Pilot_Left_YAxis = () -> (Math.pow(m_Pilot_Controller.getRawAxis(1), 3));
  private final DoubleSupplier m_Pilot_Right_XAxis = () -> (m_Pilot_Controller.getRawAxis(4));

  //F310_Copilot Variables
  /*
  private static Joystick m_Logitech_F310_Copilot = new Joystick(1);
  private final JoystickButton m_F310_Copilot_A = new JoystickButton(m_Logitech_F310_Copilot, 1);
  private final JoystickButton m_F310_Copilot_B = new JoystickButton(m_Logitech_F310_Copilot, 2);
  private final JoystickButton m_F310_Copilot_X = new JoystickButton(m_Logitech_F310_Copilot, 3);
  private final JoystickButton m_F310_Copilot_Y = new JoystickButton(m_Logitech_F310_Copilot, 4);
  private final JoystickButton m_F310_Copilot_LeftBumper = new JoystickButton(m_Logitech_F310_Copilot, 5);
  private final JoystickButton m_F310_Copilot_RightBumper = new JoystickButton(m_Logitech_F310_Copilot, 6);
  private final JoystickButton m_F310_Copilot_BACK = new JoystickButton(m_Logitech_F310_Copilot, 7);
  private final JoystickButton m_F310_Copilot_START = new JoystickButton(m_Logitech_F310_Copilot, 8);
  private final DoubleSupplier m_F310_Copilot_Left_XAxis = () -> (m_Logitech_F310_Copilot.getRawAxis(0));
  private final DoubleSupplier m_F310_Copilot_Left_YAxis = () -> (m_Logitech_F310_Copilot.getRawAxis(1));
  private final DoubleSupplier m_F310_Copilot_Right_XAxis = () -> (m_Logitech_F310_Copilot.getRawAxis(4));
  */

  //Xbox Variables:
  /*
  private static Joystick m_Xbox = new Joystick(2);
  private final JoystickButton m_Xbox_A = new JoystickButton(m_Xbox, 1);
  private final JoystickButton m_Xbox_B = new JoystickButton(m_Xbox, 2);
  private final DoubleSupplier m_Xbox_Left_XAxis = () -> (Math.pow(m_Xbox.getRawAxis(0), 3));
  private final DoubleSupplier m_Xbox_Left_YAxis = () -> (Math.pow(m_Xbox.getRawAxis(1), 3));
  private final DoubleSupplier m_Xbox_Right_XAxis = () -> (m_Xbox.getRawAxis(4));
  */

  SendableChooser<Double> m_angleChooser = new SendableChooser<>();
  
  //long scorePosition;
  
  // Limelight:
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  // Subsystems:
  private final SwerveDriveSubsystem m_swerveDrivetrain = new SwerveDriveSubsystem();
  private final ElevatorIntakeSubsystem m_ElevatorIntakeSubsystem = new ElevatorIntakeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  // Commands:
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_swerveDrivetrain);
  private final Command m_Pilot_swerveCommand = new SwerveDriveCommand(m_Pilot_Left_XAxis, m_Pilot_Left_YAxis, m_Pilot_Right_XAxis, m_swerveDrivetrain, m_angleChooser, m_ElevatorSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Command scoreCones()
  {
    /*
    scorePosition = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentGrid").getInteger(-1);
    double travelTicks = (((double)scorePosition )* 26400) + 3000;
    SmartDashboard.putNumber("TravelTicks", travelTicks);
    */

    return new InstantCommand(()-> {new WaitCommand(0.001);})
    .andThen(new DriveStraightCommand(m_swerveDrivetrain, m_angleChooser, 5000, 0.08))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 0.2, 0));    //10000
    //.andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5))
    /*
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 57500, 0.15, 0))    //84000
    .andThen(new WaitCommand(1.5))
    //.andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 8000, 0.08, 0))
    .andThen(new WaitCommand(1.5));
    */
    //.andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5));
  }

  /*
  public Command strafeTest()
  {
    return new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome();}, m_swerveDrivetrain)
    .andThen(new WaitCommand(1.5))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 300000, -0.15, 0));
  }
  */

  public RobotContainer() 
  {
    // Configure the button bindings
    m_swerveDrivetrain.setDefaultCommand(m_Pilot_swerveCommand);
    m_ElevatorIntakeSubsystem.setDefaultCommand(new RunCommand(()-> {m_ElevatorIntakeSubsystem.setNEOMotorSpeed(0);}, m_ElevatorIntakeSubsystem));
    configureButtonBindings();

    m_Pilot_A.onTrue(new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome();}, m_swerveDrivetrain));
    m_Pilot_B.onTrue(new InstantCommand(()-> {SwerveDriveSubsystem.gyro.reset();}, m_swerveDrivetrain));

    
    m_Pilot_LeftBumper.whileTrue(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -1));
    m_Pilot_RightBumper.whileTrue(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 1));

    m_Pilot_X.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem));
    
    /*
    m_F310_Copilot_BACK.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem, 0));
    m_F310_Copilot_START.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem, 1));
    m_F310_Copilot_LeftBumper.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem, 2));
    m_F310_Copilot_RightBumper.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem, 3));
    m_F310_Copilot_Y.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem, -1));
    */

   // m_F310_Copilot_X.whileTrue(new ManualElevatorCommand(m_ElevatorSubsystem, m_F310_Copilot_Left_YAxis));

    Command scoreCones = scoreCones();
    SmartDashboard.putData((Sendable) scoreCones);
    m_Pilot_BACK.whileTrue(scoreCones);

    m_Pilot_START.whileTrue(new AlignToAprilTagCommand(m_swerveDrivetrain, m_VisionSubsystem));

    //m_F310_Y.whileTrue(strafeTest());

    m_angleChooser.setDefaultOption("No tarmac offset", 0.0);
    m_angleChooser.addOption("Left tarmac offset", 21.0);
    m_angleChooser.addOption("Right tarmac offset", -69.0);
    SmartDashboard.putData(m_angleChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
