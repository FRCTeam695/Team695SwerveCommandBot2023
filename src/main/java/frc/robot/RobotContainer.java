// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final JoystickButton m_Pilot_LeftBumper = new JoystickButton(m_Pilot_Controller, 5);
  private final JoystickButton m_Pilot_RightBumper = new JoystickButton(m_Pilot_Controller, 6);

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

  SendableChooser<Command> m_pathChooser = new SendableChooser<>();
  SendableChooser<Double> m_secChooser = new SendableChooser<>();
  SendableChooser<Command> m_chargeStationChooser = new SendableChooser<>();
  SendableChooser<Double> m_sideChooser = new SendableChooser<>();

  //long scorePosition;
  
  private NetworkTable ntSidecar;
  private DoublePublisher currentMode;

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  // Limelight:
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  // Subsystems:
  private final ElevatorIntakeSubsystem m_ElevatorIntakeSubsystem = new ElevatorIntakeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final SwerveDriveSubsystem m_swerveDrivetrain = new SwerveDriveSubsystem(m_ElevatorSubsystem);
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  // Commands:
  private final Command m_Pilot_swerveCommand = new SwerveDriveCommand(m_Pilot_Left_XAxis, m_Pilot_Left_YAxis, m_Pilot_Right_XAxis, m_swerveDrivetrain, m_ElevatorSubsystem);

  PIDController test_pid;
  double netYaw;
  double limelightDistanceTicks;
  double idealYaw;
  double pitchMultiplier = -5.2;

  public RobotContainer() 
  {
    // Configure the button bindings
    m_swerveDrivetrain.setDefaultCommand(m_Pilot_swerveCommand);
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

    m_pathChooser.setDefaultOption("Cone/MoveAlongSide/Cube/NoBalance", HoustonAuton());
    m_pathChooser.addOption("Cone/MoveOverCharge/Balance", exitOverChargeStationAndBalance());
    //m_pathChooser.addOption("Substation Cube Path", substationCubePath());
    //m_pathChooser.addOption("Coopertition Cube Path", coopertitionCubePath());
    //m_pathChooser.addOption("Score Table Cube Path", scoreTableCubePath());
    m_pathChooser.addOption("Cone/MoveAlongSubstationSide/NoBalance", substationConePath());
    m_pathChooser.addOption("Cone/MoveAlongScoreTable/NoBalance", scoreTableConePath());
    //m_pathChooser.addOption("Substation 2 Cone Auton", substation2ConeAuton());
    //m_pathChooser.addOption("Substation 1.5 Cone Auton", substationOneAndAHalfConeAuton());
    m_pathChooser.addOption("Houston Wildcat", HoustonWildcat());
    SmartDashboard.putData("Auton Routine", m_pathChooser);

    m_sideChooser.setDefaultOption("Side: Substation", 1.0);
    m_sideChooser.addOption("Side: Score Table", -1.0);
    SmartDashboard.putData("Side Chooser", m_sideChooser);

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
    //.andThen(chargeStation)
    ;
  }

  public Command scorePreload()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
      .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, -1).withTimeout(3))
      .andThen(new InstantCommand(()->
        {
          currentMode.set(1); // cones
          m_ElevatorIntakeSubsystem.setcurrentLevel(3);
        }, m_swerveDrivetrain))
      .andThen(new RunElevatorCommand(m_ElevatorSubsystem))
      .andThen(new InstantCommand(()-> {SwerveDriveSubsystem.CancoderHome(0.5);}, m_swerveDrivetrain))
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
          ()-> (getAlliance() == 1 &&  deltaTicks >= 139000) || (getAlliance() == -1 && deltaTicks >= 114000),
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
        m_swerveDrivetrain).withTimeout(3).unless(() -> !gotCube)
      )
/*
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
          m_swerveDrivetrain).withTimeout(2).unless(() -> !gotCube)
        .alongWith(new RunElevatorCommand(m_ElevatorSubsystem)).unless(() -> !gotCube)
      )*/
 
      .andThen(new RunElevatorCommand(m_ElevatorSubsystem).unless(() -> !gotCube))

      .andThen(new WaitCommand(1))

      // if we have cube, discharge it
      .andThen(new RunElevatorIntakeCommand(m_ElevatorIntakeSubsystem, 1).unless(() -> !gotCube));

      //.andThen(new WaitCommand(1));

      // if we have (had) cube, lower elevator
      //.andThen(new RunElevatorCommand(m_ElevatorSubsystem).unless(() -> !gotCube));
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
          ()-> (getAlliance() == 1 &&  deltaTicks >= 139000) || (getAlliance() == -1 && deltaTicks >= 114000),
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
  double deltaAngleArray;
  double deltaAngleInstantaneous;
  double chargeStationState = 0;
  double averageAngle;
  double[] angleArray = new double[5];
  // Using gyro pitch or roll depending on navx2 orientation
  public Command dynamicEngageChargeStation()
  {
    return new InstantCommand(()-> {new WaitCommand(0.001);})
    .andThen
    (
      new FunctionalCommand(
      
      // init
      ()-> 
      {
        chargeStationState = 1;
        initialRobotAngle =  m_swerveDrivetrain.gyro.getPitch();

        // PID Controller for final balance
        auton_pid = new PIDController(0.0125, 0, 0);
        auton_pid.reset();
      },

      // execute
      ()-> 
      {

        // Create an average calculation of gyro angle (pitch)
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
        
        deltaAngleArray = initialRobotAngle + averageAngle;

        // Measure instantaneous pitch
        deltaAngleInstantaneous = initialRobotAngle + m_swerveDrivetrain.gyro.getPitch();

        // Move up the station quickly
        if(chargeStationState == 1)
        {
          m_swerveDrivetrain.driveSwerve(0, 0.50, 0, true);

          // Check for ascent
          if(deltaAngleArray > 16)
          {
            hasStartedAscent = true;
            chargeStationState = 2;
          }
        }

        // Move up slowly after sensing ascent
        if(chargeStationState == 2)
        {
          double Yj = MathUtil.clamp(auton_pid.calculate(deltaAngleInstantaneous, 0), -0.25, 0.25);
          m_swerveDrivetrain.driveSwerve(0, -Yj, 0, true);
        }

      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        chargeStationState = 3;
        hasStartedAscent = false;
      },

      // end condition
      ()->  /*hasStartedAscent == true && deltaAngle <= 12*/ false,

      // required subsystem
      m_swerveDrivetrain)
    );
  }

  // Variables for exiting the community over the charge station
  double chargeStationExitState = 0;

  public Command exitOverChargeStationAndBalance()
  {
    return
    (
      new WaitCommand(0.001)
    )

    // Move elevator down
    .andThen(new RunElevatorCommand(m_ElevatorSubsystem))

    // Move out of the community over charge station
    .andThen
    (
      new FunctionalCommand(
      
      // init
      ()-> 
      {
        chargeStationExitState = 1;
        initialRobotAngle =  m_swerveDrivetrain.gyro.getPitch();
        slew_rate = new SlewRateLimiter(0.5);
      },

      // execute
      ()-> 
      {

        // Check instantaneous angle
        deltaAngleInstantaneous = initialRobotAngle + m_swerveDrivetrain.gyro.getPitch();

        // Move up station
        if(chargeStationExitState == 1)
        {
          m_swerveDrivetrain.driveSwerve(0, -slew_rate.calculate(0.4), 0, true);
          if(deltaAngleInstantaneous <= -15)
          {
            chargeStationExitState = 2;
          }
        }

        // Move down station 
        if(chargeStationExitState == 2)
        {
          m_swerveDrivetrain.driveSwerve(0, -slew_rate.calculate(0.4), 0, true);
          if(deltaAngleInstantaneous >= 10)
          {
            chargeStationExitState = 3;
          }
        }

        // Level off 
        if(chargeStationExitState == 3)
        {
          m_swerveDrivetrain.driveSwerve(0, -slew_rate.calculate(0.4), 0, true);
          if(deltaAngleInstantaneous <= 2)
          {
            chargeStationExitState = 4;
          }
        }

        // Change state after passing pivot point
        if(chargeStationExitState == 4)
        {
          m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        }

      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
      },

      // end condition
      ()->  chargeStationExitState == 4,

      // required subsystem
      m_swerveDrivetrain)
    )

    // Move away from station slightly
    .andThen(
      new FunctionalCommand(
        
        // init
        ()-> 
        { 
          tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        },
  
        // execute
        ()-> 
        {
          m_swerveDrivetrain.driveSwerve(0, -0.10, 0, true);
        },
  
        // end
        interrupted-> 
        {
          m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        },
  
        // end condition
        ()-> Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0)) >= 20000,
  
        // required subsystem
        m_swerveDrivetrain)
      )

      // Move towards community and balance
      .andThen
      (
        new FunctionalCommand(
        
        // init
        ()-> 
        {
          chargeStationState = 1;
          initialRobotAngle =  m_swerveDrivetrain.gyro.getPitch();

          // PID Controller for final balance
          auton_pid = new PIDController(0.015, 0, 0);
          auton_pid.reset();
        },
  
        // execute
        ()-> 
        {

          // Measure instantaneous pitch
          deltaAngleInstantaneous = m_swerveDrivetrain.gyro.getPitch();

          // Move up the station quickly
          if(chargeStationState == 1)
          {
            m_swerveDrivetrain.driveSwerve(0, 0.40, 0, true);

            // Check for ascent
            if(deltaAngleInstantaneous > 16)
            {
              hasStartedAscent = true;
              chargeStationState = 2;
            }
          }

          // Move up slowly after sensing ascent
          if(chargeStationState == 2)
          {
            double Yj = MathUtil.clamp(auton_pid.calculate(deltaAngleInstantaneous, 0), -1, 1);
            m_swerveDrivetrain.driveSwerve(0, -Yj, 0, true);
          }
  
        },
  
        // end
        interrupted-> 
        {
          m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
          chargeStationState = 3;
          hasStartedAscent = false;
        },
  
        // end condition
        ()->  /*hasStartedAscent == true && deltaAngle <= 12*/ false,
  
        // required subsystem
        m_swerveDrivetrain)
      );
  }


  double tick_start;
  double tick_delta;
  double ticks_to_cube;                    // Calculated ticks to move towards cube
  double inches_to_ticks = 781;            // Inches to ticks multiplier - Adjust multiplier as needed
  SlewRateLimiter slew_rate;
  SlewRateLimiter slew_rate_turn;
  PIDController auton_pid;
  PIDController auton_pid_second;
  int apriltag_id = 7;
  double apriltag_yaw;
  double apriltag_pitch;
  double apriltag_area;
  boolean GotCube;
  double angleToCube;
  double autonStrafeSpeed;
  double gridExitTicks;
  double autonChargeStationAdjust;

  public Command HoustonAuton()
  {

    return(
    new WaitCommand(0.001)
    )

    // move from grid
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Begin HoustonAuton");
        m_VisionSubsystem.pv_setPipeline(0);
        m_ElevatorSubsystem.runToLevel(); // cleanup elevator down from scoring preloaded cone
        System.out.println("Move from grid.");
        tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        slew_rate = new SlewRateLimiter(0.5);
        auton_pid = new PIDController(0.02, 0, 0);
        auton_pid.reset();

        // Substation Side
        if(m_sideChooser.getSelected() == 1)
        {
          // Red Alliance
          if(getAlliance() == 1)
          {
            apriltag_id = 3;
            angleToCube = 135 - 10;
            autonStrafeSpeed = -0.5;
            gridExitTicks = 65000;
            autonChargeStationAdjust = 1;
          }

          // Blue Alliance
          else
          {
            apriltag_id = 6;
            angleToCube = -135 + 10;
            autonStrafeSpeed = 0.5;
            gridExitTicks = 65000;    // TBD value
            autonChargeStationAdjust = 1;
          }
        }

        // Score table side
        else
        {
          // Red Alliance
          if(getAlliance() == 1)
          {
            apriltag_id = 1;
            angleToCube = 170;
            autonStrafeSpeed = 0.5;
            gridExitTicks = 20000;
            autonChargeStationAdjust = -1;
          }

          // Blue Alliance
          else
          {
            apriltag_id = 8;
            angleToCube = -170;
            autonStrafeSpeed = -0.5;
            gridExitTicks = 20000;    // TBD value
            autonChargeStationAdjust = 1;
          }
        }

      },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroAngle, 0), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, -slew_rate.calculate(0.75), Zj, true);
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
      },

      // end condition
      ()-> Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0)) >= gridExitTicks,

      // required subsystem
      m_swerveDrivetrain)
    )

    // continue moving and rotate towards cube
    .andThen(
    new FunctionalCommand(
       
      // init
      ()-> 
      { 
        System.out.println("Move and rotate.");
        auton_pid = new PIDController(0.02, 0, 0);
        auton_pid.reset();
      },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroAngle, angleToCube), -0.5, 0.5);
        m_swerveDrivetrain.driveSwerve(0, -0.6, Zj, true);
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
      },

      // end condition
      ()-> (angleToCube >= 0 && m_swerveDrivetrain.gyroAngle >= angleToCube) || (angleToCube < 0 && m_swerveDrivetrain.gyroAngle <= angleToCube),

      // required subsystem
      m_swerveDrivetrain)
    )

    // set mode to cube and wait for camera to see it
    .andThen(
    new FunctionalCommand(
          
      // init
      ()-> 
      {
        System.out.println("Look for cube.");
      },

      // execute
      ()-> 
      {
      },

      // end
      interrupted-> 
      {
        currentMode.set(2);
        System.out.println("Done.");
      },

      // end condition
      ()-> (m_VisionSubsystem.pv_hasTarget() == true),

      // required subsystem
      m_swerveDrivetrain)

    )

    // adjust rotation and move to get cube right in front of intake
    .andThen(
    new FunctionalCommand(
          
      // init
      ()-> 
      { 
        System.out.println("Align to cube.");
        tick_start = tick_delta = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        auton_pid = new PIDController(0.015, 0, 0);
        auton_pid.reset();
        m_ElevatorIntakeSubsystem.runInit(-1);
      },

      // execute
      ()-> 
      {
        double Zj = -MathUtil.clamp(auton_pid.calculate(m_VisionSubsystem.pv_getYaw(), 0), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, -0.3, Zj, false);
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0,false);
      },

      // end condition
      ()-> (m_VisionSubsystem.pv_getArea() >= 30
        || m_VisionSubsystem.pv_hasTarget() == false),

      // required subsystem
      m_swerveDrivetrain)

    )

    // capture cube
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Capture cube.");
        m_VisionSubsystem.pv_setPipeline(1);
        tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0);
        gotCube = false;
      },

      // execute
      ()-> 
      {
        m_swerveDrivetrain.driveSwerve(0, -0.3, 0, false);
        gotCube = m_ElevatorIntakeSubsystem.getStallHold();
        tick_delta = Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0, false);
      },

      // end condition
      ()-> tick_delta >= 40000
        || gotCube == true,

      // required subsystem
      m_swerveDrivetrain)
    )

    // back away and rotate with cube
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Back away and rotate with cube.");
        auton_pid = new PIDController(0.02, 0, 0);
        auton_pid.reset();
        tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
      },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroAngle, 0), -0.8, 0.8);
        m_swerveDrivetrain.driveSwerve(0.1 * autonChargeStationAdjust, 0.3, Zj, true);
      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        System.out.println("Done.");
      },

      // end condition
      ()-> ((angleToCube >= 0 && m_swerveDrivetrain.gyroAngle <= 10) || (angleToCube < 0 && m_swerveDrivetrain.gyroAngle >= -10)),

      // required subsystem
      m_swerveDrivetrain)
    )

    // look for apriltag
    .andThen(
    new FunctionalCommand(
          
      // init
      ()-> 
      {
        System.out.println("Look for apriltag.");
        m_ElevatorIntakeSubsystem.setcurrentLevel(1);
        m_ElevatorSubsystem.runToLevel();
      },

      // execute
      ()-> 
      {
        System.out.println("Done.");
      },

      // end
      interrupted-> 
      {
      },

      // end condition
      ()-> (m_VisionSubsystem.at_hasTarget(apriltag_id) == true),

      // required subsystem
      m_swerveDrivetrain)
    )

    // start move to grid
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Start move to grid.");
        tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        slew_rate = new SlewRateLimiter(0.5);
        auton_pid = new PIDController(0.02, 0, 0);
        auton_pid.reset();
      },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroAngle, 0), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, slew_rate.calculate(0.75), Zj, true);
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
      },

      // end condition
      ()-> Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0)) >= 90000,

      // required subsystem
      m_swerveDrivetrain)
    )

    // continue move to grid with apriltag vector
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      {
        System.out.println("Apriltag to grid.");
        tick_delta = -1; 
      },

      // execute
      ()-> 
      {

        if (tick_delta == -1)
        {

          if (m_VisionSubsystem.at_hasTarget(apriltag_id) == false)
          {
            m_swerveDrivetrain.driveSwerve(0, 0, 0, false);
            return;
          }

          apriltag_yaw = m_VisionSubsystem.at_getYaw(apriltag_id) + m_swerveDrivetrain.gyroYaw;
          if (autonChargeStationAdjust == 1)
          {
            apriltag_yaw -= 2;
          }
          else
          {
            apriltag_yaw -= 3;
          }
          System.out.printf("YAW: %f\n", m_swerveDrivetrain.gyroYaw);

          apriltag_area = m_VisionSubsystem.at_getArea(apriltag_id);
          System.out.printf("AREA: %f\n", apriltag_area);
          
          tick_delta = 0.96 * (-62 * apriltag_area + 103) * 1000; // apriltag area:distance equation (encoder ticks)
          System.out.printf("TICKS: %f\n", tick_delta);
          
          tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();

          auton_pid = new PIDController(0.02, 0, 0.0);
          auton_pid.reset();

          m_ElevatorSubsystem.runToLevel();  // elevator down

          return;

        }

        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroYaw, apriltag_yaw), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, -0.45, Zj, false);

      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        System.out.println("Done.");
      },

      // end condition
      ()-> tick_delta != -1
        && Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0)) >= tick_delta,

      // required subsystem
      m_swerveDrivetrain)

    )

    // adjust elevator and dump cube
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        m_ElevatorIntakeSubsystem.setcurrentLevel(2);
        m_ElevatorSubsystem.runToLevel();
      },

      // execute
      ()-> 
      {

        /*
        // if elevator currently moving, just wait
        if (m_ElevatorSubsystem.getHold() == false)
        {
          return;
        }

        if (m_ElevatorSubsystem.getLevel() == 0)
        {
          m_ElevatorIntakeSubsystem.setcurrentLevel(2);
          m_ElevatorSubsystem.runToLevel();
        }
        */

      },

      // end
      interrupted-> 
      {
        m_ElevatorIntakeSubsystem.runInit(1);
        System.out.println("Done.");
      },

      // end condition
      ()-> m_ElevatorSubsystem.getLevel() == 2,

      // required subsystem
      m_swerveDrivetrain)

    )

    // elevator down and strafe to charge station
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Down and strafe.");
        m_ElevatorSubsystem.runToLevel(); // elevator down
        tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
      },

      // execute
      ()-> 
      {
        //m_swerveDrivetrain.driveSwerve(autonStrafeSpeed, -0.1, 0, true);
      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        m_ElevatorIntakeSubsystem.stop();
        System.out.println("Done.");
      },

      // end condition
      ()-> Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0)) >= 60000,

      // required subsystem
      m_swerveDrivetrain).withTimeout(3)

    )
/* 
    // engage charge station
    .andThen
    (
      new FunctionalCommand(
      
      // init
      ()-> 
      {
        chargeStationState = 1;
        initialRobotAngle =  m_swerveDrivetrain.gyro.getPitch();

        // PID Controller for final balance
        auton_pid = new PIDController(0.015, 0, 0);
        auton_pid.reset();

        auton_pid_second = new PIDController(0.05, 0, 0);
        auton_pid_second.reset();        
      },

      // execute
      ()-> 
      {

        // Measure instantaneous pitch
        deltaAngleInstantaneous = m_swerveDrivetrain.gyro.getPitch();

        // Move up the station quickly
        if(chargeStationState == 1)
        {
          double Zj = MathUtil.clamp(auton_pid_second.calculate(m_swerveDrivetrain.gyroYaw, 0), -1, 1);
          m_swerveDrivetrain.driveSwerve(0, -0.5, Zj, true);

          // Check for ascent
          if(deltaAngleInstantaneous < -16)
          {
            hasStartedAscent = true;
            chargeStationState = 2;
          }
        }

        // Move up slowly after sensing ascent
        if(chargeStationState == 2)
        {
          double Yj = MathUtil.clamp(auton_pid.calculate(deltaAngleInstantaneous, 0), -1, 1);
          m_swerveDrivetrain.driveSwerve(0, -Yj, 0, true);
        }

      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        chargeStationState = 3;
        hasStartedAscent = false;
      },

      // end condition
      ()->  false,

      // required subsystem
      m_swerveDrivetrain)
    )
*/
    ;
  }


// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************
// *****************************************************************************************************************



  public Command HoustonWildcat()
  {

    return(
    new WaitCommand(0.001)
    )

    // move from grid
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Begin HoustonWildcat");
        m_VisionSubsystem.pv_setPipeline(0);
        m_ElevatorSubsystem.runToLevel(); // cleanup elevator down from scoring preloaded cone
        slew_rate = new SlewRateLimiter(0.6);
        auton_pid = new PIDController(0.02, 0, 0);
        auton_pid.reset();
        tick_delta = 0;

        // Red Alliance
        if(getAlliance() == 1)
        {
          apriltag_id = 2;
        }

        // Blue Alliance
        else
        {
          apriltag_id = 7;
        }

      },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroAngle, -170), -.25, .25);
        m_swerveDrivetrain.driveSwerve(0, -slew_rate.calculate(0.4), Zj, true);
        tick_start = Math.abs(m_swerveDrivetrain.gyro.getPitch()) + Math.abs(m_swerveDrivetrain.gyro.getRoll());
        System.out.printf("%f %f\n", tick_delta, tick_start);
        if (tick_delta == 0)
        {
          if (tick_start >= 14.0)
          {
            tick_delta++;
          }
        }
        else if (tick_delta == 1)
        {
          if (tick_start <= 10.0)
          {
            tick_delta++;
          }
        }
        else if (tick_delta == 2)
        {
          if (tick_start >= 14.0)
          {
            tick_delta++;
          }
        }
        else if (tick_delta == 3)
        {
          if (tick_start < 10.0)
          {
            tick_delta++;
          }
        }
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
      },

      // end condition
      ()-> m_swerveDrivetrain.gyroAngle <= -170 && tick_delta == 4,

      // required subsystem
      m_swerveDrivetrain)
    )

    // look for cube
    .andThen(
    new FunctionalCommand(
          
      // init
      ()-> 
      {
        System.out.println("Look for cube.");
      },

      // execute
      ()-> 
      {
      },

      // end
      interrupted-> 
      {
        currentMode.set(2);
        System.out.println("Done.");
      },

      // end condition
      ()-> (m_VisionSubsystem.pv_hasTarget() == true),

      // required subsystem
      m_swerveDrivetrain)

    )

    // adjust rotation and move to get cube right in front of intake
    .andThen(
    new FunctionalCommand(
          
      // init
      ()-> 
      { 
        System.out.println("Align to cube.");
        tick_start = tick_delta = m_swerveDrivetrain.drive[0].getSelectedSensorPosition();
        auton_pid = new PIDController(0.015, 0, 0);
        auton_pid.reset();
        m_ElevatorIntakeSubsystem.runInit(-1);
      },

      // execute
      ()-> 
      {
        double Zj = -MathUtil.clamp(auton_pid.calculate(m_VisionSubsystem.pv_getYaw(), 0), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, -0.3, Zj, false);
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0,false);
      },

      // end condition
      ()-> (m_VisionSubsystem.pv_getArea() >= 30
        || m_VisionSubsystem.pv_hasTarget() == false),

      // required subsystem
      m_swerveDrivetrain)

    )

    // capture cube
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Capture cube.");
        m_VisionSubsystem.pv_setPipeline(1);
        tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0);
        gotCube = false;
      },

      // execute
      ()-> 
      {
        m_swerveDrivetrain.driveSwerve(0, -0.3, 0, false);
        gotCube = m_ElevatorIntakeSubsystem.getStallHold();
        tick_delta = Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0));
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0, false);
      },

      // end condition
      ()-> tick_delta >= 30000
        || gotCube == true,

      // required subsystem
      m_swerveDrivetrain)
    )

    // move to grid
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        slew_rate = new SlewRateLimiter(1); // JPK was 0.5
        auton_pid = new PIDController(0.02, 0, 0);
        auton_pid.reset();
        apriltag_area = -1;
      },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroAngle, 0), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, slew_rate.calculate(0.5), Zj, true);
        if (m_VisionSubsystem.at_hasTarget(apriltag_id) == true)
        {
          tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0);
          apriltag_yaw = m_VisionSubsystem.at_getYaw(apriltag_id);
          apriltag_area = m_VisionSubsystem.at_getArea(apriltag_id);      
          System.out.printf("Found AT yaw=%f area=%f.", apriltag_yaw, apriltag_area);
        }
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
      },

      // end condition
      ()-> m_swerveDrivetrain.gyro.getPitch() < -5.0,

      // required subsystem
      m_swerveDrivetrain)
    )

    // look for apriltag
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        System.out.println("Look for Apriltag.");
        m_ElevatorIntakeSubsystem.setcurrentLevel(1);
        m_ElevatorSubsystem.runToLevel();
      },

      // execute
      ()-> 
      {

        if (m_VisionSubsystem.at_hasTarget(apriltag_id) == true && apriltag_area == -1)
        {
          tick_start = m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0);
          apriltag_yaw = m_VisionSubsystem.at_getYaw(apriltag_id);
          apriltag_area = m_VisionSubsystem.at_getArea(apriltag_id);      
          System.out.printf("Found AT yaw=%f area=%f.", apriltag_yaw, apriltag_area);
        }

      },

      // end
      interrupted-> 
      {
      },

      // end condition
      ()-> apriltag_area != -1,

      // required subsystem
      m_swerveDrivetrain).withTimeout(1)
    )

    // continue move to grid with apriltag vector
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      {
        System.out.println("Apriltag to grid.");
        apriltag_yaw = m_VisionSubsystem.at_getYaw(apriltag_id) + m_swerveDrivetrain.gyroYaw - 5;
        tick_delta = (0.94 * (-62 * apriltag_area + 103) * 1000) - 12500; // apriltag area:distance equation (encoder ticks)         
        auton_pid = new PIDController(0.02, 0, 0.0);
        auton_pid.reset();
    },

      // execute
      ()-> 
      {
        double Zj = MathUtil.clamp(auton_pid.calculate(m_swerveDrivetrain.gyroYaw, apriltag_yaw), -1, 1);
        m_swerveDrivetrain.driveSwerve(0, -0.3, Zj, false);
      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        //m_ElevatorIntakeSubsystem.setcurrentLevel(0);
        //m_ElevatorSubsystem.runToLevel();
        System.out.println("Done.");
      },

      // end condition
      ()-> Math.abs(tick_start - m_swerveDrivetrain.drive[0].getSelectedSensorPosition(0)) >= tick_delta,

      // required subsystem
      m_swerveDrivetrain).unless(() -> apriltag_area == -1)

    )
/*
    // elevator to 2
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      {
        System.out.println("Elevator to 2.");
      },

      // execute
      ()-> 
      {
        if (m_ElevatorSubsystem.getLevel() == 0)
        {
          m_ElevatorIntakeSubsystem.setcurrentLevel(2);
          m_ElevatorSubsystem.runToLevel();  
        }
      },

      // end
      interrupted-> 
      {
        System.out.println("Done.");
      },

      // end condition
      ()-> m_ElevatorSubsystem.getLevel() == 2,

      // required subsystem
      m_swerveDrivetrain).unless(() -> apriltag_area == -1)

    )
*/
    // dump cube
    .andThen(
    new FunctionalCommand(
      
      // init
      ()-> 
      { 
        m_ElevatorIntakeSubsystem.runInit(1);
        m_ElevatorIntakeSubsystem.setcurrentLevel(0);
        m_ElevatorSubsystem.runToLevel();
      },

      // execute
      ()-> 
      {
      },

      // end
      interrupted-> 
      {
        m_ElevatorIntakeSubsystem.stop();
        System.out.println("Done.");
      },

      // end condition
      ()-> false,

      // required subsystem
      m_swerveDrivetrain).withTimeout(0.25).unless(() -> apriltag_area == -1)

    )

    // engage charge station
    .andThen
    (
      new FunctionalCommand(
      
      // init
      ()-> 
      {
        System.out.println("Engage charge station.");
        chargeStationState = 1;
        initialRobotAngle =  m_swerveDrivetrain.gyro.getPitch();

        // PID Controller for final balance
        auton_pid = new PIDController(0.01, 0, 0);
        auton_pid.reset();

        auton_pid_second = new PIDController(0.05, 0, 0);
        auton_pid_second.reset();        
      },

      // execute
      ()-> 
      {

        // Measure instantaneous pitch
        deltaAngleInstantaneous = m_swerveDrivetrain.gyro.getPitch();

        // Move up the station quickly
        if(chargeStationState == 1)
        {
          double Zj = MathUtil.clamp(auton_pid_second.calculate(m_swerveDrivetrain.gyroYaw, 0), -1, 1);
          m_swerveDrivetrain.driveSwerve(0, -0.5, Zj, true);

          // Check for ascent
          if(deltaAngleInstantaneous < -16)
          {
            hasStartedAscent = true;
            chargeStationState = 2;
          }
        }

        // Move up slowly after sensing ascent
        if(chargeStationState == 2)
        {
          double Yj = MathUtil.clamp(auton_pid.calculate(deltaAngleInstantaneous, 0), -1, 1);
          m_swerveDrivetrain.driveSwerve(0, -Yj, 0, true);
        }

      },

      // end
      interrupted-> 
      {
        m_swerveDrivetrain.driveSwerve(0, 0, 0, true);
        chargeStationState = 3;
        hasStartedAscent = false;
      },

      // end condition
      ()->  false,

      // required subsystem
      m_swerveDrivetrain)
    )

    ;
  }




}
