// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Drive motor CAN IDs
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 4;
        
        // Drive characterization & tuning values
        public static final double ksVolts = 0.198;   //0.169
        public static final double kvVoltSecondsPerMeter = 2.86;  //2.24\
        public static final double kaVoltSecondsSquaredPerMeter = 0.365;  //0.0435
        public static final double kPDriveVel = 0.025;  //2.4 8/14 2.24 Tuning to get better PIDF response
        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);  
        public static final int kSlotDriveLeftPID = 0;
        public static final int kSlotDriveRightPID = 0;

        // Drive system wheels & gearing values
        public static final int kEncoderCPR = 42;
        public static final double kGearRatio = 10.71;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
        public static final double kDistanceMetersPerEncoderUnits = ((double) (Math.PI*kWheelDiameterMeters)/(kGearRatio));
        public static final double kVelocityMetersPerSecondPerEncoderUnits = ((double) (Math.PI*kWheelDiameterMeters)/(60*kGearRatio));

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    
}
