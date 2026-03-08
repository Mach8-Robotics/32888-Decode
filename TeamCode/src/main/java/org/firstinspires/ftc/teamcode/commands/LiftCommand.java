package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.FootSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;

import java.util.concurrent.TimeUnit;

public class LiftCommand extends CommandBase {
    private FootSubsystem footSubsystem;

    public LiftCommand(FootSubsystem footSubsystem) {
        this.footSubsystem = footSubsystem;
        addRequirements(footSubsystem);
    }

    @Override
    public void execute(){
        footSubsystem.lift();
    }

}
