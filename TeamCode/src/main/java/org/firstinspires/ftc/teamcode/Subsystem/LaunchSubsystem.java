package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LaunchSubsystem extends SubsystemBase {
    private DcMotorEx LeftMotor, RightMotor;
    public LaunchSubsystem(DcMotorEx LeftMotor,DcMotorEx RightMotor){
        this.LeftMotor = LeftMotor;
        this.RightMotor = RightMotor;
    }
    public void Launch(double power){
        LeftMotor.setPower(power);
        RightMotor.setPower(power);
    }
}