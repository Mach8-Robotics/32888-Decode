package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
    private DcMotor rightCatapultMotor, leftCatapultMotor;

    @Override
    public void initialize(){
        //controlAssignments
        driverOp = new GamepadEx(gamepad1);
        rightCatapultMotor = hardwareMap.get(DcMotor.class,"catapult1");
        leftCatapultMotor = hardwareMap.get(DcMotor.class, "catapult2");
        launchSubsystem = new LaunchSubsystem(rightCatapultMotor,leftCatapultMotor);


        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new LaunchCommand(launchSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new RetractCommand(launchSubsystem));



//        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(()->launchSubsystem.Launch()));
//
//        driverOp.getGamepadButton(GamepadKeys.Button.A).whenReleased(
//                new InstantCommand(()->launchSubsystem.Hold()));


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

