// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
// flapper  private final JoystickButton m_Pilot_Y = new JoystickButton(m_Pilot_Controller, 4);
  private final JoystickButton m_Pilot_LeftBumper = new JoystickButton(m_Pilot_Controller, 5);
  private final JoystickButton m_Pilot_RightBumper = new JoystickButton(m_Pilot_Controller, 6);
// elevator  private final JoystickButton m_Pilot_BACK = new JoystickButton(m_Pilot_Controller, 7);
// elevator  private final JoystickButton m_Pilot_START = new JoystickButton(m_Pilot_Controller, 8);
  
  // with pow3:
  //private final DoubleSupplier m_Pilot_Left_XAxis = () -> (Math.pow(m_Pilot_Controller.getRawAxis(0), 3));
  //private final DoubleSupplier m_Pilot_Left_YAxis = () -> (Math.pow(m_Pilot_Controller.getRawAxis(1), 3));

  // without pow3:
  private final DoubleSupplier m_Pilot_Left_XAxis = () -> (m_Pilot_Controller.getRawAxis(0));
  private final DoubleSupplier m_Pilot_Left_YAxis = () -> (m_Pilot_Controller.getRawAxis(1));
  
  private final DoubleSupplier m_Pilot_Right_XAxis = () -> (m_Pilot_Controller.getRawAxis(4));

  // POV Buttons
  private final POVButton m_Pilot_POV_UP = new POVButton(m_Pilot_Controller, 0);
  private final POVButton m_Pilot_POV_RIGHT = new POVButton(m_Pilot_Controller, 90);
  private final POVButton m_Pilot_POV_DOWN = new POVButton(m_Pilot_Controller, 180);
  private final POVButton m_Pilot_POV_LEFT = new POVButton(m_Pilot_Controller, 270);

  // Constant sped suppliers
  private final DoubleSupplier m_ConstantSpeed = () -> (0.4);
  private final DoubleSupplier m_ConstantSpeedInverted = () -> (-0.4);
  private final DoubleSupplier m_NoSpeed = () -> (0);  

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

  SendableChooser<Command> m_pathChooser = new SendableChooser<>();
  SendableChooser<Double> m_secChooser = new SendableChooser<>();
  SendableChooser<Command> m_chargeStationChooser = new SendableChooser<>();

  //long scorePosition;
  
  private NetworkTable ntSidecar;
  private DoublePublisher currentMode;

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  // Limelight:
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  // Subsystems:
  private final ElevatorIntakeSubsystem m_ElevatorIntakeSubsystem = new ElevatorIntakeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  private final SwerveDriveSubsystem m_swerveDrivetrain = new SwerveDriveSubsystem(m_ElevatorSubsystem);
  
  // Commands:
//  private final ExampleCommand m_autoCommand = new ExampleCommand(m_swerveDrivetrain);
  private final Command m_Pilot_swerveCommand = new SwerveDriveCommand(m_Pilot_Left_XAxis, m_Pilot_Left_YAxis, m_Pilot_Right_XAxis, m_swerveDrivetrain, m_ElevatorSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Command scoreCones()
  {
    /*
    scorePosition = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentGrid").getInteger(-1);
    double travelTicks = (((double)scorePosition )* 26400) + 3000;
    SmartDashboard.putNumber("TravelTicks", travelTicks);
    */

    return new InstantCommand(()-> {new WaitCommand(0.001);})
    .andThen(new DriveStraightCommand(m_swerveDrivetrain, 5000, 0.08))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, 0.2, 0));    //10000
    //.andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5))
    /*
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, 57500, 0.15, 0))    //84000
    .andThen(new WaitCommand(1.5))
    //.andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, 8000, 0.08, 0))
    .andThen(new WaitCommand(1.5));
    */
    //.andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 0.7).withTimeout(1.5));
  }

  /*
  public Command strafeTest()
  {
    return new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome();}, m_swerveDrivetrain)
    .andThen(new WaitCommand(1.5))
    .andThen(new StrafeToTargetCommand(m_swerveDrivetrain, m_VisionSubsystem, 300000, -0.15, 0));
  }
  */

  public RobotContainer() 
  {
    // Configure the button bindings
    m_swerveDrivetrain.setDefaultCommand(m_Pilot_swerveCommand);
    //m_ElevatorIntakeSubsystem.setDefaultCommand(new RunCommand(()-> {m_ElevatorIntakeSubsystem.setDirection(0);}, m_ElevatorIntakeSubsystem));
    configureButtonBindings();

    m_Pilot_A.onTrue(new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome(0.2);}, m_swerveDrivetrain));
    m_Pilot_B.onTrue(new InstantCommand(()-> {SwerveDriveSubsystem.gyro.reset();}, m_swerveDrivetrain));

    m_Pilot_POV_UP.whileTrue(new SwerveDriveCommand(m_NoSpeed, m_ConstantSpeedInverted, m_NoSpeed, m_swerveDrivetrain, m_ElevatorSubsystem));
    m_Pilot_POV_RIGHT.whileTrue(new SwerveDriveCommand(m_ConstantSpeed, m_NoSpeed, m_NoSpeed, m_swerveDrivetrain, m_ElevatorSubsystem));
    m_Pilot_POV_DOWN.whileTrue(new SwerveDriveCommand(m_NoSpeed, m_ConstantSpeed, m_NoSpeed, m_swerveDrivetrain, m_ElevatorSubsystem));
    m_Pilot_POV_LEFT.whileTrue(new SwerveDriveCommand(m_ConstantSpeedInverted, m_NoSpeed, m_NoSpeed, m_swerveDrivetrain, m_ElevatorSubsystem));
    
    m_Pilot_LeftBumper.onTrue(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -1));
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

//    Command scoreCones = scoreCones();
    //SmartDashboard.putData((Sendable) scoreCones);
//    m_Pilot_BACK.whileTrue(scoreCones);

//    m_Pilot_START.whileTrue(new AlignToAprilTagCommand(m_swerveDrivetrain, m_VisionSubsystem));

    //m_F310_Y.whileTrue(strafeTest());

    m_pathChooser.setDefaultOption("Substation Cube Path", substationCubePath());
    m_pathChooser.addOption("Coopertition Cube Path", coopertitionCubePath());
    m_pathChooser.addOption("Score Table Cube Path", scoreTableCubePath());
    m_pathChooser.addOption("Substation Cone Path", substationConePath());
    m_pathChooser.addOption("Score Table Cone Path", scoreTableConePath());
    //m_pathChooser.addOption("Substation Cube PIROUETTE", substationCubePathPirouette());
    m_pathChooser.addOption("Substation 2 Cone Auton", substation2ConeAuton());
    m_pathChooser.addOption("Substation 1.5 Cone Auton", substationOneAndAHalfConeAuton());
    SmartDashboard.putData("Auton Routine", m_pathChooser);

    m_secChooser.setDefaultOption("0", 0.0);
    m_secChooser.addOption("1", 1.0);
    m_secChooser.addOption("2", 2.0);
    m_secChooser.addOption("3", 3.0);
    m_secChooser.addOption("4", 4.0);
    m_secChooser.addOption("5", 5.0);
    SmartDashboard.putData("Auton Start Delay", m_secChooser);

    m_chargeStationChooser.setDefaultOption("Do Not Engage Station", doNothing());
    m_chargeStationChooser.addOption("Engage Station", dynamicEngageChargeStation());
    SmartDashboard.putData("Auton Charge Station", m_chargeStationChooser);

    ntSidecar = NetworkTableInstance.getDefault().getTable("sidecar695");
    currentMode = ntSidecar.getDoubleTopic("currentIntakeMode").publish();
    
    //CameraServer.startAutomaticCapture();
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

  public double getAlliance()
  {
    DriverStation.Alliance allianceColor;

    allianceColor = DriverStation.getAlliance();

    if(allianceColor == DriverStation.Alliance.Red)
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }

  public Command getAutonomousCommand() 
  {
    Command selectedPath = m_pathChooser.getSelected();
    Command chargeStation = m_chargeStationChooser.getSelected();

    scheduler.removeComposedCommand(selectedPath);
    scheduler.removeComposedCommand(chargeStation);

    return new WaitCommand(m_secChooser.getSelected())
    .andThen(scorePreload())
    .andThen(selectedPath)
    .andThen(chargeStation);
  }

  public Command scorePreload()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen(new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome(0.2);}, m_swerveDrivetrain))
      .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -1).withTimeout(3))
      .andThen(new RunElevatorCommand(m_ElevatorSubsystem))
      .andThen(new WaitCommand(0.5))
      .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 1).withTimeout(0.5));
  }

  double initialRobotYaw;
  double initialTicks;
  double deltaTicks;
  boolean metDesiredAngle;

  public Command substationCubePathPirouette()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen
      (
        new FunctionalCommand(
        ()-> 
        {
          initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        },
        ()-> 
        {
          m_swerveDrivetrain.driveSpline(0, 0.40, initialTicks, 180);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 300000,    //121000
        m_swerveDrivetrain)
      )
      .andThen(new WaitCommand(5))
      .andThen
      (
        new FunctionalCommand(
        ()-> 
        {
          initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        },
        ()-> 
        {
          m_swerveDrivetrain.driveSpline(0, -0.40, initialTicks, 0);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 300000,    //121000
        m_swerveDrivetrain)
      )
      .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();});
  }

  double xSpeed;
  double ySpeed;
  double ticksToCube;
  boolean gotCube = false;

  public Command substation2ConeAuton()
  { 
    return new InstantCommand(()-> {new WaitCommand(0.001);})

      // set mode to cube
      .andThen(new InstantCommand(() ->{currentMode.set(2);}))

      // move downfield from grid
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(0.70, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 97500,
          m_swerveDrivetrain)

        // lower elevator while moving downfield
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))

      )

      // continue moving downfield with 90 degree CW rotation
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveSpline((0.01)*(getAlliance()), 0.40, initialTicks, 90*(getAlliance()));
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> (getAlliance() == 1 &&  deltaTicks >= 134000) || (getAlliance() == -1 && deltaTicks >= 104000),
          m_swerveDrivetrain)
      )

      // move towards cube to attempt intake (terminates if we acquire cube or move too far)
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStrafe((-0.25)*(getAlliance()), initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
            gotCube = m_ElevatorIntakeSubsystem.getStallHold();
            System.out.println(deltaTicks);
            System.out.println(gotCube);
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 70000 || gotCube == true,
          m_swerveDrivetrain)
        .raceWith
        (
          new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -1)
        )
      )
      .andThen(new WaitCommand(0.250).unless(() -> !gotCube))
      // if we have cube, turn 90 CCW and start towards grid
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveSpline((0.50)*(getAlliance()), 0, initialTicks, 0);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> (getAlliance() == 1 && deltaTicks >= 10000) || (getAlliance() == -1 && deltaTicks >= 15000),
          m_swerveDrivetrain).unless(() -> !gotCube)
      )

      // if we have cube, continue towards grid
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw = 0;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(-0.80, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 130000,
          m_swerveDrivetrain).unless(() -> !gotCube)
      )

      // if we have cube, we should now be in apriltag range, so align to the tag
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            if(m_VisionSubsystem.getPitch() >= -8)
            {
              ySpeed = -0.25;
            }
            else
            {
              ySpeed = 0;
            }
  
            if(getAlliance() == 1)
            {
              if(m_VisionSubsystem.getYaw() >= 22 || m_VisionSubsystem.hasTarget() == false)
              {
                xSpeed = -0.15;
              }
              else
              {
                xSpeed = 0;
              }
            }
            else
            {
              if(m_VisionSubsystem.getYaw() <= 21 && m_VisionSubsystem.hasTarget() == true)
              {
                xSpeed = 0.11;
              }
              else
              {
                xSpeed = 0;
              }
            }

            m_swerveDrivetrain.driveSpline(xSpeed, ySpeed, initialTicks, 0);
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> (m_VisionSubsystem.getPitch() <= -7),
        m_swerveDrivetrain).unless(() -> !gotCube)
      )

      // if we have cube, finish move up to grid and extend elevator
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(-0.2, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 7500,
          m_swerveDrivetrain).unless(() -> !gotCube)
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem)).unless(() -> !gotCube)
      )

      // if we have cube, discharge it
      .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 1).withTimeout(1).unless(() -> !gotCube))

      // if we have (had) cube, lower elevator
      .andThen(new RunElevatorCommand(m_ElevatorSubsystem).unless(() -> !gotCube));
  }

  public Command substationOneAndAHalfConeAuton()
  { 
    return new InstantCommand(()-> {new WaitCommand(0.001);})

      // set mode to cube
      .andThen(new InstantCommand(() ->{currentMode.set(2);}))

      // move downfield from grid
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(0.70, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 97500,
          m_swerveDrivetrain)

        // lower elevator while moving downfield
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))

      )

      // continue moving downfield with 90 degree CW rotation
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveSpline((0.01)*(getAlliance()), 0.40, initialTicks, 90*(getAlliance()));
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> (getAlliance() == 1 &&  deltaTicks >= 134000) || (getAlliance() == -1 && deltaTicks >= 104000),
          m_swerveDrivetrain)
      )

      // move towards cube to attempt intake (terminates if we acquire cube or move too far)
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStrafe((-0.25)*(getAlliance()), initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 60000,
          m_swerveDrivetrain)
        .raceWith
        (
          new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -1)
        )
      )
      .andThen
      (
        new FunctionalCommand(
          ()-> {},
          ()-> 
          {
            m_swerveDrivetrain.rotateInPlace(0);
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> (getAlliance() == 1 && m_swerveDrivetrain.gyroYaw <= 5) || (getAlliance() == -1 && m_swerveDrivetrain.gyroYaw >= -5),
          m_swerveDrivetrain)
      )
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveSpline((-0.20)*(getAlliance()), -0.40, initialTicks, 0);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 80000,
          m_swerveDrivetrain)
      );
  }
  
  public Command substationCubePath()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(0.45, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 150370,
          m_swerveDrivetrain)
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))
      )
      .andThen(new WaitCommand(0.1))
      .andThen
      (
        new FunctionalCommand(
        ()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();},
        ()-> 
        {
          m_swerveDrivetrain.driveStrafe((-0.45)*(getAlliance()), initialRobotYaw, initialTicks);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 60150,
        m_swerveDrivetrain)
      )
      .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();});
  }

  public Command substationConePath()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(0.45, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 150370,
          m_swerveDrivetrain)
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))
      )
      .andThen(new WaitCommand(0.1))
      .andThen
      (
        new FunctionalCommand(
        ()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();},
        ()-> 
        {
          m_swerveDrivetrain.driveStrafe((-0.45)*(getAlliance()), initialRobotYaw, initialTicks);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 71425,
        m_swerveDrivetrain)
      )
      .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();});
  }

  public Command scoreTableCubePath()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(0.45, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 150370,
          m_swerveDrivetrain)
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))
      )
      .andThen(new WaitCommand(0.1))
      .andThen
      (
        new FunctionalCommand(
        ()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();},
        ()-> 
        {
          m_swerveDrivetrain.driveStrafe((0.45)*(getAlliance()), initialRobotYaw, initialTicks);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 60150,
        m_swerveDrivetrain)
      )
      .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();});
  }

  public Command scoreTableConePath()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen
      (
        new FunctionalCommand( 
          ()-> 
          {
            initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
            initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
          },
          ()-> 
          {
            m_swerveDrivetrain.driveStraight(0.45, initialRobotYaw, initialTicks);
            deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
          },
          interrupted-> 
          {
            for(int lp=0; lp<4; lp++)
            {
              m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
              m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
            }
          },
          ()-> deltaTicks >= 150370,
          m_swerveDrivetrain)
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))
      )
      .andThen(new WaitCommand(0.1))
      .andThen
      (
        new FunctionalCommand(
        ()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();},
        ()-> 
        {
          m_swerveDrivetrain.driveStrafe((0.45)*(getAlliance()), initialRobotYaw, initialTicks);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 71425,
        m_swerveDrivetrain)
      )
      .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();});
  }

  public Command coopertitionCubePath()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
    .andThen
    (
      new FunctionalCommand( 
        ()-> 
        {
          initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
          initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
        },
        ()-> 
        {
          m_swerveDrivetrain.driveStraight(0.08, initialRobotYaw, initialTicks);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 3760,
        m_swerveDrivetrain)
      .alongWith(new RunElevatorCommand(m_ElevatorSubsystem))
    )
    .andThen(new WaitCommand(0.1))
    .andThen
    (
      new FunctionalCommand(
      ()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();},
      ()-> 
      {
        m_swerveDrivetrain.driveStrafe((0.20)*(getAlliance()), initialRobotYaw, initialTicks);
        deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
      },
      interrupted-> 
      {
        for(int lp=0; lp<4; lp++)
        {
          m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
          m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
        }
      },
      ()-> deltaTicks >= 71425,
      m_swerveDrivetrain)
    )
    .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();})
    .andThen
    (
      new FunctionalCommand( 
        ()-> 
        {
          initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
          initialRobotYaw =  m_swerveDrivetrain.gyroYaw;
        },
        ()-> 
        {
          m_swerveDrivetrain.driveStraight(0.45, initialRobotYaw, initialTicks);
          deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
        },
        ()-> deltaTicks >= 150370,
        m_swerveDrivetrain)
    )
    .andThen(new WaitCommand(0.1))
    .andThen
    (
      new FunctionalCommand(
      ()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();},
      ()-> 
      {
        m_swerveDrivetrain.driveStrafe((-0.45)*(getAlliance()), initialRobotYaw, initialTicks);
        deltaTicks = Math.abs(initialTicks - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
      },
      interrupted-> 
      {
        for(int lp=0; lp<4; lp++)
        {
          m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
          m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
        }
      },
      ()-> deltaTicks >= 60150,
      m_swerveDrivetrain)
    )
    .andThen(()-> {initialTicks = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();});
  }

  public Command doNothing()
  {
    return new WaitCommand(0.001);
  }

  double initialRobotAngle;
  boolean hasStartedAscent = false;
  double deltaAngle;
  double chargeStationState = 0;
  double averageAngle;
  double[] angleArray = new double[5];
  // Using gyro pitch or roll depending on navx2 orientation
  public Command dynamicEngageChargeStation()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
    .andThen
    (
      new FunctionalCommand
      (
        ()->           
        {
          chargeStationState = 1;
          initialRobotAngle =  m_swerveDrivetrain.gyro.getPitch();
//          SmartDashboard.putNumber("Charge Station State", chargeStationState);
        },
        ()->
        {          
          for(int i = 3; i >= 0; i--)
          {
            angleArray[i+1] = angleArray[i];
          }

          angleArray[0] = m_swerveDrivetrain.gyro.getPitch();

          double total = 0;

          for(int i=0; i < angleArray.length; i++)
          {
            total = total + angleArray[i];
          }

          averageAngle = total / angleArray.length;  
          
          SmartDashboard.putNumber("AverageAngle", averageAngle);

          deltaAngle = Math.abs(initialRobotAngle - averageAngle);

          if(chargeStationState == 1)
          {
            m_swerveDrivetrain.driveStraight(-0.50, initialRobotYaw, initialTicks);
            if(deltaAngle > 16)
            {
              hasStartedAscent = true;
              chargeStationState = 2;
            }
          }

          if(chargeStationState == 2)
          {
            m_swerveDrivetrain.driveStraight(-0.16, initialRobotYaw, initialTicks);
          }
          
          //SmartDashboard.putNumber("Charge Station State", chargeStationState);
        },
        interrupted-> 
        {
          for(int lp=0; lp<4; lp++)
          {
            m_swerveDrivetrain.steer[lp].set(ControlMode.PercentOutput, 0);
            m_swerveDrivetrain.drive[lp].set(ControlMode.PercentOutput, 0);
          }
          chargeStationState = 3;
          //SmartDashboard.putNumber("Charge Station State", chargeStationState);
          hasStartedAscent = false;
        },
        ()-> hasStartedAscent == true && deltaAngle <= 11,
        m_swerveDrivetrain)
    );
  }
}
