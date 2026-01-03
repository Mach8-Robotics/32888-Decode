package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuSubsystem extends SubsystemBase {
    private IMU imu;
    private RevHubOrientationOnRobot orientationOnRobot;

    public ImuSubsystem(IMU imu,RevHubOrientationOnRobot orientationOnRobot){
        this.imu = imu;
        this.orientationOnRobot =orientationOnRobot;
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public double getYawDeg(){return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}

}
