package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.WingSubsystem;

import java.util.concurrent.TimeUnit;

public class WingDownCommand extends CommandBase {
    private WingSubsystem wingSubsystem;

    public WingDownCommand(WingSubsystem wingSubsystem) {
        this.wingSubsystem = wingSubsystem;
    }

    @Override
    public void initialize(){
        wingSubsystem.wingsdown();
    }

    @Override
    public boolean isFinished(){return false;}

}
