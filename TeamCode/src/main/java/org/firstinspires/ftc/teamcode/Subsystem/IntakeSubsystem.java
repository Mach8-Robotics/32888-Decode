package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotorEx motor;
    public IntakeSubsystem(DcMotorEx motor){
        this.motor = motor;
    }
    public void intake(double power){
        motor.setPower(power);
    }
}
