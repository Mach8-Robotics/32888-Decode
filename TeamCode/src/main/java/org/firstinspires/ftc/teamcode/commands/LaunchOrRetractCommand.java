package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class LaunchOrRetractCommand extends CommandBase {
    private LaunchSubsystem launchSubsystem;
    private BooleanSupplier shouldLaunch, shouldRetract;

    public LaunchOrRetractCommand(LaunchSubsystem launchSubsystem, BooleanSupplier launch, BooleanSupplier retract) {
        this.launchSubsystem = launchSubsystem;
        shouldLaunch = launch;
        shouldRetract = retract;
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute(){
        if( shouldLaunch.getAsBoolean()){
            launchSubsystem.Launch();
        }
        else if(shouldRetract.getAsBoolean()){
            launchSubsystem.Retract();
        }
        else {
            launchSubsystem.Hold();
        }

    }
}
