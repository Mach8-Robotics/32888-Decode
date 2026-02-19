package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubSystem;
import org.firstinspires.ftc.teamcode.Subsystem.ImuSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.RangeSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchOrRetractCommand;
import org.firstinspires.ftc.teamcode.commands.MonitorRangeCommand;


@TeleOp(name = "Raptor: Competition TeleOp")
public class RobotCentricTeleOpControlRaptor extends CommandOpMode{
    private GamepadEx driverOp, secondaryOp;
    private DriveSubSystem driveSubSystem;
    private LaunchSubsystem launchSubsystem;
    private ImuSubsystem imuSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private DcMotor rightCatapultMotor, leftCatapultMotor;
    private Motor frontleft,frontright,backleft,backright;
    private DcMotor intakeMotor;

    private DistanceSensor distanceSensor;
    private Servo led;
    private RangeSubsystem rangeSubsystem;

    @Override
    public void initialize() {

        driverOp = new GamepadEx(gamepad1);
        secondaryOp = new GamepadEx(gamepad2);

        rightCatapultMotor = hardwareMap.get(DcMotor.class,"catapult1");
        leftCatapultMotor = hardwareMap.get(DcMotor.class, "catapult2");
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor,leftCatapultMotor);
        launchSubsystem.setDefaultCommand(new LaunchOrRetractCommand(launchSubsystem,
                ()-> driverOp.gamepad.right_trigger>0.2,
                ()-> secondaryOp.gamepad.right_bumper));

        frontleft = new Motor(hardwareMap,"left_front_drive");
        frontright = new Motor(hardwareMap,"right_front_drive");
        backleft = new Motor(hardwareMap,"left_back_drive");
        backright = new Motor(hardwareMap,"right_back_drive");
        driveSubSystem = new DriveSubSystem(frontleft,backleft,frontright,backright);

        intakeMotor = hardwareMap.get(DcMotor.class,"intake");
        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        intakeSubsystem.setDefaultCommand(
            new IntakeCommand(intakeSubsystem,
                () -> secondaryOp.gamepad.left_trigger >0.2,
                ()-> secondaryOp.gamepad.left_bumper));

        //sets IMU parameters and defines reference varibles
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        // defines Imu
        imuSubsystem = new ImuSubsystem(hardwareMap.get(IMU.class, "imu"),orientationOnRobot);

        distanceSensor=hardwareMap.get(DistanceSensor.class,"distance_sensor");
        led=hardwareMap.get(Servo.class,"led");
        rangeSubsystem=new RangeSubsystem(distanceSensor,led);
        rangeSubsystem.setDefaultCommand(new MonitorRangeCommand(rangeSubsystem));

        //Drive commands
        driveSubSystem.setDefaultCommand(new DriveCommand(driveSubSystem,
                                                        driverOp::getLeftX,
                                                        driverOp::getLeftY,
                                                        driverOp::getRightX,
                                                        imuSubsystem::getYawDeg,
                                                false));
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

