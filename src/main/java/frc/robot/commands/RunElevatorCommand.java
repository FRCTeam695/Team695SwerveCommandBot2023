// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RunElevatorCommand extends CommandBase 
{
  private final ElevatorSubsystem m_ElevatorSubsystem;

  /*
  double maxSpeed;
  double maxTicks;

  double initSpeed = 0.02;
  double downSpeed;
  */
  
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

    /*
    double stageTwo = maxTicks * 0.66;
    double adjUpSpeed;
    double adjDownSpeed;
    boolean finished = false;
    */

    double maxTime = 3.0;                     // maximum seconds to allow command to run
    double rotations = 100;                   // number of rotations to run
    double minSpeed = 0.1;                    // start speed
    double maxSpeed = 1.0;                    // plateau speed
    double maxTicks = 2048 * rotations;       // total tick count for target rotations (falcon 500)
    double plateauStart = 0.2 * maxTicks;     // threshold tick count when ramp up is complete
    double plateauEnd = 0.8 * maxTicks;       // threshold tick count to begin ramp down
    double tickstart, ticks;
    double targetSpeed;

    // create a timer to enforce maxTime
    Timer et = new Timer();
    et.reset();
    et.start();

    // get start encoder ticks (saves time by not resetting encoders)
    tickstart = m_ElevatorSubsystem.getPosition();

    // loop to run until maxTime
    while(et.get() < maxTime)
    {

      // get current ticks from start ticks
      ticks = m_ElevatorSubsystem.getPosition() - tickstart;

      // if at tick target, exit loop
      if (ticks >= maxTicks)
      {
        break;
      }

      // before plateau speed ramp up
      if (ticks < plateauStart)
      {
        targetSpeed = maxSpeed * ticks / plateauStart;
      }

      // at plateau speed constant at max
      else if (ticks < plateauEnd)
      {
        targetSpeed = maxSpeed;
      }

      // after plateau speed ramp down
      else
      {
        targetSpeed = maxSpeed  - maxSpeed * (ticks - plateauEnd) / (maxTicks - plateauEnd);
      }

      // ensure target speed is at least at minimum (does not go to zero and stop prematurely)
      if (targetSpeed < minSpeed)
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
