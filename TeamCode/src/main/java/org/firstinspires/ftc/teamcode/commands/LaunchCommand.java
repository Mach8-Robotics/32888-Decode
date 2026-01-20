package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;

import java.util.concurrent.TimeUnit;

public class LaunchCommand extends CommandBase {
    private LaunchSubsystem launchSubsystem;
    private Timing.Timer commandTimer;

    public LaunchCommand(LaunchSubsystem launchSubsystem) {
        this.launchSubsystem = launchSubsystem;
        addRequirements(launchSubsystem);
        commandTimer = new Timing.Timer(100, TimeUnit.MILLISECONDS);
        commandTimer.start();
    }

    @Override
    public void execute(){

        launchSubsystem.Launch();
    }

    @Override
    public boolean isFinished(){
        return commandTimer.done();
    }
}
