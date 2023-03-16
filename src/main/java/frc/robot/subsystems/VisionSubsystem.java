// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  //PhotonCamera camera = new PhotonCamera("OV5647");
//  PhotonTrackedTarget target;

  boolean hasTargets;
  double pitch;
  double yaw;
  double area;

  public VisionSubsystem() 
  {
  }

  /*public void getBestTarget()
  {
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();

    if(hasTargets)
    {
      var targlist = result.getTargets();
      var numTarg = targlist.size();
      target = result.getBestTarget();
      //target = targlist.get(0);
      //if (1 < numTarg)
      //{
      //  double bestarea = 0;
      //  double bestyaw = target.getYaw();
        //target = targ
      //}
    }
  }*/

  public boolean hasTarget()
  {
    boolean ret = false;
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if(tv == 1)
    {
      ret = true;
    }
    return(ret);
  }

  public double getPitch()
  {
    return pitch;
  }

  public double getYaw()
  {
    return yaw;
  }

  public double getArea()
  {
    return area;
  }

  @Override
  public void periodic() 
  {
    if (hasTarget() == true)
    {
      yaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      pitch = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    }
    else
    {
      yaw = 0;
      pitch = 0;
      area = 0;
    }

//    SmartDashboard.putBoolean("LT", hasTarget());
//    SmartDashboard.putNumber("LTPitch", pitch);
//    SmartDashboard.putNumber("LTYaw", yaw);
//    SmartDashboard.putNumber("LTArea",area);  

  }
}
