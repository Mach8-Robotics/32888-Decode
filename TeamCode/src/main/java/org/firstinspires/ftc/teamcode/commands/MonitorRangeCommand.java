package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.RangeSubsystem;

import java.util.concurrent.TimeUnit;

public class MonitorRangeCommand extends CommandBase {
    private RangeSubsystem rangeSubsystem;


    public MonitorRangeCommand(RangeSubsystem rangeSubsystem) {
        this.rangeSubsystem = rangeSubsystem;
        addRequirements(rangeSubsystem);
    }

    @Override
    public void execute(){
        rangeSubsystem.CheckRange();
    }
    @Override
    public boolean isFinished(){return false;}


}
