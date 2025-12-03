package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SimpleDCMotorSubsystem extends SubsystemBase {
    private DcMotor dcmotor;
    public SimpleDCMotorSubsystem(DcMotor dcmotor){
        this.dcmotor = dcmotor;
    }

    public void runMotor(){
        dcmotor.setPower(1);
    }

    public void stopMotor(){
        dcmotor.setPower(0);
    }
}
