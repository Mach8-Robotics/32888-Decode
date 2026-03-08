package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FootSubsystem extends SubsystemBase {
    private final DcMotor motor;
    private static final DcMotorSimple.Direction FOOT_DOWN  = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction FOOT_UP = DcMotorSimple.Direction.FORWARD;
    private DcMotorSimple.Direction currentdirection;


    public FootSubsystem(DcMotor motor) {
        this.motor = motor;
        this.motor.setDirection(FOOT_UP);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setPower(0.0);
        currentdirection = FOOT_UP;
    }

    public void lift(){
        motor.setDirection(FOOT_DOWN);
        motor.setPower(0.9);
        currentdirection = FOOT_DOWN;

    }




    public void retract(){
        motor.setDirection(FOOT_UP);
        motor.setPower(0.9);
        currentdirection = FOOT_UP;
    }


    public void hold(){
        if(currentdirection == FOOT_DOWN){
            motor.setPower(0.1);
        }
        else{
            motor.setPower(0.0);
        }

    }

}

