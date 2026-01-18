package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    public IntakeSubsystem(DcMotorEx motor) {
        this.motor = motor;
        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void Intake() {
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(.9);
    }



    public void Stop() {
        motor.setPower(0);
    }
    public void Outtake() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(.9);
    }
}

