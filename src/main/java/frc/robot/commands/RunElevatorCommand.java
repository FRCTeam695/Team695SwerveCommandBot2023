// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.spi.BreakIteratorProvider;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RunElevatorCommand extends CommandBase 
{
  private final ElevatorSubsystem m_ElevatorSubsystem;

  int newLevel;
  
  public RunElevatorCommand(ElevatorSubsystem elevatorSubsystem) 
  {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    long currentLevel = NetworkTableInstance.getDefault().getTable("sidecar695").getEntry("currentLevel").getInteger(-1);

    m_ElevatorSubsystem.runToLevel((int)currentLevel);
  
    return;

    /*
    double stageTwo = maxTicks * 0.66;
    double adjUpSpeed;
    double adjDownSpeed;
    boolean finished = false;
    */
    
    /*
    double targetEncoderValue = 0;
    double direction = 1.0;
    double pos = m_ElevatorSubsystem.getPosition();

    if (pos >= 96000)
    {
      switch(level)
      {
        case "top":
          break;
        case "middle":
          targetEncoderValue = 49000;
          direction = -1.0;
          break;
        case "bottom":
          targetEncoderValue = 0;
          direction = -1.0;
          break;
      }
    }
    else if (pos >= 49000 && pos < 96000)
    {
      switch(level)
      {
        case "top":
          targetEncoderValue = 96000;
          direction = 1.0;
          break;
        case "middle":
          break;
        case "bottom":
          targetEncoderValue = 0;
          direction = -1.0;
          break;
      }
    }
    else
    {
      switch(level)
      {
        case "top":
          targetEncoderValue = 96000;
          direction = 1.0;
          break;
        case "middle":
          targetEncoderValue = 49000;
          direction = 1.0;
          break;
        case "bottom":
          break;
      }
    }

    double maxTime = 7.0;                     // maximum seconds to allow command to run
    //double rotations = relativeRotations;                   // number of rotations to run
    double minSpeed = 0.1 * direction;                    // start speed
    double maxSpeed = 1 * direction;                    // plateau speed
    //double maxTicks = targetEncoderValue;       // total tick count for target rotations (falcon 500)
    double moveTicks = Math.abs(pos) - targetEncoderValue;
    double plateauStart = pos + 0.2*direction*moveTicks;     // threshold tick count when ramp up is complete
    double plateauEnd = pos + 0.8*direction*moveTicks;       // threshold tick count to begin ramp down
    double targetSpeed;

    // create a timer to enforce maxTime
    Timer et = new Timer();
    et.reset();
    et.start();

    // loop to run until maxTime
    while(et.get() < maxTime)
    {
      pos = m_ElevatorSubsystem.getPosition();
      if(direction == 1.0)
      {
        if(pos >= targetEncoderValue)
        {
          break;
        }
        // before plateau speed ramp up
        if (pos < plateauStart)
        {
          targetSpeed = maxSpeed * pos / plateauStart;
        }

        // at plateau speed constant at max
        else if (pos < plateauEnd)
        {
          targetSpeed = maxSpeed;
        }

        // after plateau speed ramp down
        else
        {
          targetSpeed = maxSpeed  - maxSpeed * (pos - plateauEnd) / (targetEncoderValue - plateauEnd);
        }
      }
      else
      {
        if(pos <= targetEncoderValue)
        {
          break;
        }
        // before plateau speed ramp up
        if (pos > plateauStart)
        {
          targetSpeed = maxSpeed * pos / plateauStart;
        }

        // at plateau speed constant at max
        else if (pos > plateauEnd)
        {
          targetSpeed = maxSpeed;
        }

        // after plateau speed ramp down
        else
        {
          targetSpeed = maxSpeed  - maxSpeed * (pos - plateauEnd) / (targetEncoderValue - plateauEnd);
        }
      }      

      // ensure target speed is at least at minimum (does not go to zero and stop prematurely)
      if (Math.abs(targetSpeed) < Math.abs(minSpeed))
      {
        targetSpeed = minSpeed;
      }

      // send target speed to motor
      m_ElevatorSubsystem.setSpeed(targetSpeed);

      // delay 5 msec to not saturate CAN
      Timer.delay(0.005);
    }

    // stop motor and timer
    m_ElevatorSubsystem.setSpeed(0.0);
    et.stop();

    /*

    while(m_ElevatorSubsystem.getPosition() <= maxTicks && finished == false)
    {
      if(m_ElevatorSubsystem.getPosition() < stageTwo)
      {
        if(m_ElevatorSubsystem.getSpeed() < maxSpeed)
        {
          adjUpSpeed = initSpeed + 0.00002;
          m_ElevatorSubsystem.setSpeed(adjUpSpeed);
          initSpeed = adjUpSpeed;
          //System.out.println("ACCELERATING ELEVATOR");
        }
        else
        {
          m_ElevatorSubsystem.setSpeed(maxSpeed);
          //System.out.println("PLATEAUING ELEVATOR");
        }
      }
      else
      {
        adjDownSpeed = downSpeed - 0.00002;
        m_ElevatorSubsystem.setSpeed(adjDownSpeed);
        downSpeed = adjDownSpeed;
        if(m_ElevatorSubsystem.getSpeed() <= 0)
        {
          finished = true;
        }
        //System.out.println("DECELERATING ELEVATOR");
      }
      Timer.delay();
    }
      m_ElevatorSubsystem.setSpeed(0);
      System.out.println("STOP ELEVATOR");
    */

  }





  /*
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

//    double stageTwo = maxTicks * 0.66;
//    double adjUpSpeed;
//    double adjDownSpeed;
//    boolean finished = false;
    
    double targetEncoderValue = 0;
    double direction = 1.0;
    double pos = m_ElevatorSubsystem.getPosition();

    if (pos >= 96000)
    {
      switch(level)
      {
        case "top":
          break;
        case "middle":
          targetEncoderValue = 49000;
          direction = -1.0;
          break;
        case "bottom":
          targetEncoderValue = 0;
          direction = -1.0;
          break;
      }
    }
    else if (pos >= 49000 && pos < 96000)
    {
      switch(level)
      {
        case "top":
          targetEncoderValue = 96000;
          direction = 1.0;
          break;
        case "middle":
          break;
        case "bottom":
          targetEncoderValue = 0;
          direction = -1.0;
          break;
      }
    }
    else
    {
      switch(level)
      {
        case "top":
          targetEncoderValue = 96000;
          direction = 1.0;
          break;
        case "middle":
          targetEncoderValue = 49000;
          direction = 1.0;
          break;
        case "bottom":
          break;
      }
    }

    double maxTime = 7.0;                     // maximum seconds to allow command to run
    //double rotations = relativeRotations;                   // number of rotations to run
    double minSpeed = 0.1 * direction;                    // start speed
    double maxSpeed = 1 * direction;                    // plateau speed
    //double maxTicks = targetEncoderValue;       // total tick count for target rotations (falcon 500)
    double moveTicks = Math.abs(pos) - targetEncoderValue;
    double plateauStart = pos + 0.2*direction*moveTicks;     // threshold tick count when ramp up is complete
    double plateauEnd = pos + 0.8*direction*moveTicks;       // threshold tick count to begin ramp down
    double targetSpeed;

    // create a timer to enforce maxTime
    Timer et = new Timer();
    et.reset();
    et.start();

    // loop to run until maxTime
    while(et.get() < maxTime)
    {
      pos = m_ElevatorSubsystem.getPosition();
      if(direction == 1.0)
      {
        if(pos >= targetEncoderValue)
        {
          break;
        }
        // before plateau speed ramp up
        if (pos < plateauStart)
        {
          targetSpeed = maxSpeed * pos / plateauStart;
        }

        // at plateau speed constant at max
        else if (pos < plateauEnd)
        {
          targetSpeed = maxSpeed;
        }

        // after plateau speed ramp down
        else
        {
          targetSpeed = maxSpeed  - maxSpeed * (pos - plateauEnd) / (targetEncoderValue - plateauEnd);
        }
      }
      else
      {
        if(pos <= targetEncoderValue)
        {
          break;
        }
        // before plateau speed ramp up
        if (pos > plateauStart)
        {
          targetSpeed = maxSpeed * pos / plateauStart;
        }

        // at plateau speed constant at max
        else if (pos > plateauEnd)
        {
          targetSpeed = maxSpeed;
        }

        // after plateau speed ramp down
        else
        {
          targetSpeed = maxSpeed  - maxSpeed * (pos - plateauEnd) / (targetEncoderValue - plateauEnd);
        }
      }      

      // ensure target speed is at least at minimum (does not go to zero and stop prematurely)
      if (Math.abs(targetSpeed) < Math.abs(minSpeed))
      {
        targetSpeed = minSpeed;
      }

      // send target speed to motor
      m_ElevatorSubsystem.setSpeed(targetSpeed);

      // delay 5 msec to not saturate CAN
      Timer.delay(0.005);
    }

    // stop motor and timer
    m_ElevatorSubsystem.setSpeed(0.0);
    et.stop();


  }
*/










  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    //return m_ElevatorSubsystem.getPosition() >= maxTicks;
  }
}
