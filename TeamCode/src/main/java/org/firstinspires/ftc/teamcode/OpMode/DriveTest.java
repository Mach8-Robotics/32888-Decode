package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubSystem;
import org.firstinspires.ftc.teamcode.Subsystem.ImuSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;


    @TeleOp(name = "Drive Test")
    public class DriveTest extends CommandOpMode{
        private GamepadEx driverOp;
        private DriveSubSystem driveSubSystem;
        private Motor frontleft,frontright,backleft,backright;
        private ImuSubsystem imuSubsystem;
        @Override
        public void initialize(){
            //controlAssignments
            driverOp = new GamepadEx(gamepad1);
            frontleft = hardwareMap.get(Motor.class,"left_front_drive");
            frontright = hardwareMap.get(Motor.class,"right_front_drive");
            backleft = hardwareMap.get(Motor.class,"left_back_drive");
            backright = hardwareMap.get(Motor.class,"right_back_drive");
            driveSubSystem = new DriveSubSystem(frontleft,frontright,backleft,backright);
            //sets IMU parameters and defines reference varibles
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
            logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
            usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
            // defines Imu
            imuSubsystem = new ImuSubsystem(hardwareMap.get(IMU.class, "imu"),orientationOnRobot);
            //Drive commands
            driveSubSystem.setDefaultCommand(new DriveCommand(driveSubSystem,driverOp::getLeftX, driverOp::getLeftY, driverOp::getRightX,imuSubsystem::getYawDeg,true));
        }

        public void runOpMode(){
            initialize();
            waitForStart();

            while(!isStopRequested() && opModeIsActive()){
                run();
            }
            reset();
        }
    }

