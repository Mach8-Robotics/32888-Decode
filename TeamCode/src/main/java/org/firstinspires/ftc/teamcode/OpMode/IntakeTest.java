package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystem.ImuSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;


@TeleOp(name = "Tomcat: Intake Test")
@Disabled
public class IntakeTest extends CommandOpMode{
    private GamepadEx driverOp;
    private IntakeSubsystem intakeSubsystem;
    private DcMotor intakeMotor;

    @Override
    public void initialize(){
        //controlAssignments
        driverOp = new GamepadEx(gamepad1);
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");
        intakeSubsystem = new IntakeSubsystem(intakeMotor);


        intakeSubsystem.setDefaultCommand(
                new IntakeCommand(intakeSubsystem,
                        () -> driverOp.gamepad.left_trigger >0.2,
                        ()-> driverOp.gamepad.left_bumper));
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

