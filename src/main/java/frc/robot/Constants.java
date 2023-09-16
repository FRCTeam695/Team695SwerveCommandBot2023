// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** 2112
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    // Falcon encoder count per 360 degrees of steering rotation
    // Falcon 500 is 2048 counts per 360 degrees of revolution
    // Mk4i steering ratio is 150/7:1
    public static double talon_mk4i_360_count = 2048 * 150 / 7;
}
