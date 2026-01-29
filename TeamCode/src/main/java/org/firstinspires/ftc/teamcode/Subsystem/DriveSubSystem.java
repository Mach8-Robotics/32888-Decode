package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;


public class DriveSubSystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor frontLeft,backLeft,frontRight,backRight;
    public DriveSubSystem(Motor frontLeft,Motor backLeft,Motor frontRight,Motor backRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft.setInverted(false);
        this.backLeft.setInverted(false);
        this.frontRight.setInverted(true);
        this.backRight.setInverted(true);
        this.drive = new MecanumDrive(false,frontLeft,frontRight,backLeft,backRight);
    }

    public DriveSubSystem(Motor frontLeft,Motor backLeft,Motor frontRight,Motor backRight, boolean isTomcat){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        if(!isTomcat) {
            this.frontLeft.setInverted(false);
            this.backLeft.setInverted(false);
            this.frontRight.setInverted(true);
            this.backRight.setInverted(true);
        }
        else {
            this.frontLeft.setInverted(true);
            this.backLeft.setInverted(true);
            this.frontRight.setInverted(false);
            this.backRight.setInverted(false);
        }
        this.drive = new MecanumDrive(false,frontLeft,frontRight,backLeft,backRight);
    }
    public void drive(double strafe,double forward,double turn,double gyroAngle, boolean fieldCentric){

        if(fieldCentric){
            drive.driveFieldCentric(strafe,forward,turn, gyroAngle);
        }
        else {
            drive.driveRobotCentric(strafe,forward,turn);
        }
    }
}
