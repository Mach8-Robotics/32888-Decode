package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;

import java.util.concurrent.TimeUnit;

public class RetractCommand extends CommandBase {
    private LaunchSubsystem launchSubsystem;
    private Timing.Timer commandTimer;

    public RetractCommand(LaunchSubsystem launchSubsystem) {
        this.launchSubsystem = launchSubsystem;
        commandTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize(){
        commandTimer.start();
        launchSubsystem.Retract();
    }

    @Override
    public boolean isFinished(){return commandTimer.done();}

    @Override
    public void end(boolean interrupted){launchSubsystem.Hold();}
}
