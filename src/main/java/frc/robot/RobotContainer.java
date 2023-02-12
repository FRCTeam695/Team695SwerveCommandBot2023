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

  //F310 Variables:
  private static Joystick m_Logitech_F310 = new Joystick(0);
  private final JoystickButton m_F310_A = new JoystickButton(m_Logitech_F310, 1);
  private final JoystickButton m_F310_B = new JoystickButton(m_Logitech_F310, 2);
  private final JoystickButton m_F310_X = new JoystickButton(m_Logitech_F310, 3);
  private final JoystickButton m_F310_Y = new JoystickButton(m_Logitech_F310, 4);
  private final JoystickButton m_F310_LeftBumper = new JoystickButton(m_Logitech_F310, 5);
  private final JoystickButton m_F310_RightBumper = new JoystickButton(m_Logitech_F310, 6);
  private final JoystickButton m_F310_BACK = new JoystickButton(m_Logitech_F310, 7);
  private final JoystickButton m_F310_START = new JoystickButton(m_Logitech_F310, 8);
  private final DoubleSupplier m_F310_Left_XAxis = () -> (Math.pow(m_Logitech_F310.getRawAxis(0), 3));
  private final DoubleSupplier m_F310_Left_YAxis = () -> (Math.pow(m_Logitech_F310.getRawAxis(1), 3));
  private final DoubleSupplier m_F310_Right_XAxis = () -> (m_Logitech_F310.getRawAxis(4));

  //Xbox Variables:
  private static Joystick m_Xbox = new Joystick(1);
  private final JoystickButton m_Xbox_A = new JoystickButton(m_Xbox, 1);
  private final JoystickButton m_Xbox_B = new JoystickButton(m_Xbox, 2);
  private final DoubleSupplier m_Xbox_Left_XAxis = () -> (Math.pow(m_Xbox.getRawAxis(0), 3));
  private final DoubleSupplier m_Xbox_Left_YAxis = () -> (Math.pow(m_Xbox.getRawAxis(1), 3));
  private final DoubleSupplier m_Xbox_Right_XAxis = () -> (m_Xbox.getRawAxis(4));

  SendableChooser<Double> m_angleChooser = new SendableChooser<>();
  
  // Limelight:
  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(RobotMainNetworkTableInstance, 0);

  // Subsystems:
  private final SwerveDriveSubsystem m_swerveDrivetrain = new SwerveDriveSubsystem();
  private final ElevatorIntakeSubsystem m_ElevatorIntakeSubsystem = new ElevatorIntakeSubsystem();
  private final FlapManipulatorSubsystem m_FlapManipulatorSubsystem = new FlapManipulatorSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final VictorSPXSubsystem m_VictorSPXSubsystem = new VictorSPXSubsystem();
  
  // Commands:
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_swerveDrivetrain);
  private final Command m_F310_swerveCommand = new SwerveDriveCommand(m_F310_Left_XAxis, m_F310_Left_YAxis, m_F310_Right_XAxis, m_swerveDrivetrain, m_angleChooser);
  private final Command m_Xbox_swerveCommand = new SwerveDriveCommand(m_Xbox_Left_XAxis, m_Xbox_Left_YAxis, m_Xbox_Right_XAxis, m_swerveDrivetrain, m_angleChooser);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Command scoreCones()
  {
    return new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome();}, m_swerveDrivetrain)
    .andThen(new WaitCommand(1))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 3000, 0.08, 0))    //10000
    .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 60000, 0.25, 0))    //84000
    .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, m_angleChooser, 8000, 0.08, 0))
    .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5));
  }

  public RobotContainer() 
  {
    // Configure the button bindings
    m_swerveDrivetrain.setDefaultCommand(m_F310_swerveCommand);
    m_FlapManipulatorSubsystem.setDefaultCommand(new RunCommand(()-> {m_FlapManipulatorSubsystem.setSpeed(0);}, m_FlapManipulatorSubsystem));   // TODO: Set to zero
    m_ElevatorIntakeSubsystem.setDefaultCommand(new RunCommand(()-> {m_ElevatorIntakeSubsystem.setNEOMotorSpeed(0);}, m_ElevatorIntakeSubsystem));
// JPK:  WHY???    m_ElevatorSubsystem.setDefaultCommand(new RunCommand(()-> {m_ElevatorSubsystem.setSpeed(0);}, m_ElevatorSubsystem));
    configureButtonBindings();

    m_F310_A.onTrue(new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome();}, m_swerveDrivetrain));
    m_F310_B.onTrue(new InstantCommand(()-> {SwerveDriveSubsystem.gyro.reset();}, m_swerveDrivetrain));

    //m_F310_LeftBumper.whileTrue(new RunVictorCommand(m_VictorSPXSubsystem, 1));   // Release
    //m_F310_RightBumper.whileTrue(new RunVictorCommand(m_VictorSPXSubsystem, -1));   // Intake

    m_F310_LeftBumper.whileTrue(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -0.7));
    m_F310_RightBumper.whileTrue(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7));

    m_F310_Y.whileTrue(new RunFlapManipulatorCommand(m_FlapManipulatorSubsystem, true));
    m_F310_Y.whileFalse(new RunFlapManipulatorCommand(m_FlapManipulatorSubsystem, false));

    m_F310_BACK.whileTrue(new RunElevatorCommand(m_ElevatorSubsystem));

    Command scoreCones = scoreCones();
    SmartDashboard.putData((Sendable) scoreCones);
    //m_F310_X.whileTrue(scoreCones);

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
