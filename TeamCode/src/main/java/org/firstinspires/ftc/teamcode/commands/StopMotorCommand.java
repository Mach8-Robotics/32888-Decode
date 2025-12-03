package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.SimpleDCMotorSubsystem;

public class StopMotorCommand extends CommandBase {

    private SimpleDCMotorSubsystem motorSubsystem;

    public StopMotorCommand(SimpleDCMotorSubsystem motorSubsystem){
        this.motorSubsystem = motorSubsystem;
        addRequirements(motorSubsystem);
    }
    @Override
    public void execute(){
        motorSubsystem.stopMotor();
    }
}
