package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class AutoIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;


    public AutoIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void execute(){
        intakeSubsystem.Intake(true);

    }
}
