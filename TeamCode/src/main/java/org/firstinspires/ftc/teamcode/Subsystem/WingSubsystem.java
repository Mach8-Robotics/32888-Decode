package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;

public class WingSubsystem extends SubsystemBase {
    private GamepadEx secondaryOp;
    private Servo rightAxonServo;
    private Servo leftAxonServo;
private static final double LIFT_OFF_POSITION =0.0;
    private static final double LIFT_ON_POSITION =0.5;
    public WingSubsystem(Servo leftAxonServo, Servo rightAxonServo) {
    this.leftAxonServo=leftAxonServo;
    this.rightAxonServo=rightAxonServo;
    this.leftAxonServo.setDirection(Servo.Direction.REVERSE);
    this.rightAxonServo.setDirection(Servo.Direction.FORWARD);
    }


    public void CheckButton() {
        if(secondaryOp.gamepad.left_stick_button){
           leftAxonServo.setPosition(LIFT_ON_POSITION);
           rightAxonServo.setPosition(LIFT_ON_POSITION);

        }

        else if(secondaryOp.gamepad.right_stick_button){
            leftAxonServo.setPosition(LIFT_OFF_POSITION);
            rightAxonServo.setPosition(LIFT_OFF_POSITION);
        }
    }

    public void wingsdown(){
            leftAxonServo.setPosition(LIFT_ON_POSITION);
            rightAxonServo.setPosition(LIFT_ON_POSITION);
    }
    public void wingsup(){
        leftAxonServo.setPosition(LIFT_OFF_POSITION);
        rightAxonServo.setPosition(LIFT_OFF_POSITION);
    }





}

