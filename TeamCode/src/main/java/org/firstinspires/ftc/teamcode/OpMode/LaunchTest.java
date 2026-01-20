package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.commands.HoldLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;


@TeleOp(name = "Tomcat: Launch Test")
public class LaunchTest extends CommandOpMode{
    private GamepadEx driverOp;
    private LaunchSubsystem launchSubsystem;
    private DcMotor rightCatapultMotor;

    @Override
    public void initialize(){
        //controlAssignments
        driverOp = new GamepadEx(gamepad1);
        rightCatapultMotor = hardwareMap.get(DcMotor.class,"right_catapult");
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor);


        launchSubsystem.setDefaultCommand(
                new HoldLaunchCommand(launchSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new LaunchCommand(launchSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new RetractCommand(launchSubsystem));

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

