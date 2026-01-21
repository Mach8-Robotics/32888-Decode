package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FootSubsystem extends SubsystemBase {
    private DcMotor footMotor;

    public FootSubsystem(DcMotor footMotor){
        this.footMotor=footMotor;
        this.footMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
//public void