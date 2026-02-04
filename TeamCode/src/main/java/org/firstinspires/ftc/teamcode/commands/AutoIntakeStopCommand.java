package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class AutoIntakeStopCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;


    public AutoIntakeStopCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void execute(){
        intakeSubsystem.Stop();

    }
}
