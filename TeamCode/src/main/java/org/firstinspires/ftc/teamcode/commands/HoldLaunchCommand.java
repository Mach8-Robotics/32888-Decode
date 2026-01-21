package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;

import java.util.function.BooleanSupplier;

public class HoldLaunchCommand extends CommandBase {
    private LaunchSubsystem launchSubsystem;

    public HoldLaunchCommand(LaunchSubsystem launchSubsystem) {
        this.launchSubsystem = launchSubsystem;
        addRequirements(launchSubsystem);

    }

    @Override
    public void initialize(){launchSubsystem.Hold();}
}
