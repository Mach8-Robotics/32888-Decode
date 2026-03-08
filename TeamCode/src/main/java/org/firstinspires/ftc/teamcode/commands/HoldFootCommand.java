package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.FootSubsystem;

public class HoldFootCommand extends CommandBase {
    private FootSubsystem footSubsystem;

    public HoldFootCommand(FootSubsystem footSubsystem) {
        this.footSubsystem = footSubsystem;
        addRequirements(footSubsystem);
    }

    @Override
    public void execute(){
        footSubsystem.hold();
    }

}
