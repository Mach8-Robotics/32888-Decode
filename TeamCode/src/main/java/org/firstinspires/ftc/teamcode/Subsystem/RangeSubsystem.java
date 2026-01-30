package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSubsystem extends SubsystemBase {
   private DistanceSensor distanceSensor;
   private Servo led;
private static final double LED_OFF_POSITION =0.0;
    private static final double LED_GREEN_POSITION =0.48;
    public RangeSubsystem(DistanceSensor sensor,Servo led) {
    distanceSensor=sensor;
    this.led=led;
    this.led.setPosition(LED_OFF_POSITION);}

    public void CheckRange() {
        double distance=distanceSensor.getDistance(DistanceUnit.INCH);
        if(distance>5.0 && distance<12.0){
            led.setPosition(LED_GREEN_POSITION);
        }
        else{
            led.setPosition(LED_OFF_POSITION);
        }
    }





}

