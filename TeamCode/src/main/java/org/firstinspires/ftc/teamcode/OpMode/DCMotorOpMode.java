package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.SimpleDCMotorSubsystem;
import org.firstinspires.ftc.teamcode.commands.RunMotorCommand;
import org.firstinspires.ftc.teamcode.commands.StopMotorCommand;
@TeleOp
public class DCMotorOpMode extends CommandOpMode {
    private DcMotor motor;
    private SimpleDCMotorSubsystem motorSubsystem;
    private GamepadEx driverOp;

    @Override
    public void initialize(){
        driverOp = new GamepadEx(gamepad1);
        motor = hardwareMap.get(DcMotor.class,"motor");

         motorSubsystem = new SimpleDCMotorSubsystem(motor);

                // button controls
        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RunMotorCommand(motorSubsystem));
        driverOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new StopMotorCommand(motorSubsystem));
        driverOp.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new RunMotorCommand(motorSubsystem),new StopMotorCommand(motorSubsystem));

    }
}
