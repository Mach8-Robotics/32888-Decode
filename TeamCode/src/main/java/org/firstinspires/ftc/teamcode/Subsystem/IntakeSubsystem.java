package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor motor;
    private static final double MOTOR_STOPPED_POWER = 0.0;
    private static final double RUNNING_MOTOR_POWER = 0.90;


    public IntakeSubsystem(DcMotor motor) {
        this.motor = motor;
        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void Intake(Boolean reversed) {

        if(reversed){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        motor.setPower(RUNNING_MOTOR_POWER);
    }



    public void Stop() {
        motor.setPower(MOTOR_STOPPED_POWER);
    }

}

