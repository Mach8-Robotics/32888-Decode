package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LaunchSubsystem extends SubsystemBase {
    private DcMotor LeftMotor, RightMotor;
    private static final double LAUNCH_POWER = -1.0;
    private static final double RETRACT_POWER = 1.0;
    private static final double HOLD_POWER = 0.2;


    public LaunchSubsystem(DcMotor RightMotor, DcMotor LeftMotor){
        this.LeftMotor = LeftMotor;
        this.RightMotor = RightMotor;

        // assume right motor is catapult1
        this.RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Launch(){
        LeftMotor.setPower(LAUNCH_POWER);
        RightMotor.setPower(LAUNCH_POWER);
    }
    public void Retract(){
        LeftMotor.setPower(RETRACT_POWER);
        RightMotor.setPower(RETRACT_POWER);
    }
    public void Hold(){
        LeftMotor.setPower(HOLD_POWER);
        RightMotor.setPower(HOLD_POWER);
    }
}