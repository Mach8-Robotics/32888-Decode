package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;


public class DriveSubSystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor frontLeft,backLeft,frontRight,backRight;
    public DriveSubSystem(Motor frontLeft,Motor backLeft,Motor frontRight,Motor backRight){
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
                this.backRight = backRight;
                this.drive = new MecanumDrive(false,frontLeft,backLeft,frontRight,backRight);
    }
    public void drive(double strafe,double forward,double turn,double gyroAngle, boolean fieldCentric){
        frontLeft.setInverted((false);
        backLeft.setInverted(false);
        frontRight.setInverted(false);
        backRight.setInverted(false);
        drive.driveRobotCentric(strafe,forward,turn);
    }
}
