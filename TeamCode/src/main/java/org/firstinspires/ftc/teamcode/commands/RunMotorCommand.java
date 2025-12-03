package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.SimpleDCMotorSubsystem;

public class RunMotorCommand extends CommandBase {

    private SimpleDCMotorSubsystem motorSubsystem;

    public RunMotorCommand(SimpleDCMotorSubsystem motorSubsystem){
        this.motorSubsystem = motorSubsystem;
        addRequirements(motorSubsystem);
    }
    @Override
    public void execute(){
        motorSubsystem.runMotor();
    }
}
