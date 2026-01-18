package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    
    private BooleanSupplier runForward;
    private BooleanSupplier runReversed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier runForward, BooleanSupplier runReversed) {
        this.intakeSubsystem = intakeSubsystem;
        this.runForward = runForward;
        this.runReversed = runReversed;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void execute(){
        if(runForward.getAsBoolean()){
            intakeSubsystem.Intake(false);
        }
        else if(runReversed.getAsBoolean()){
            intakeSubsystem.Intake(true);
        }
        else {
            intakeSubsystem.Stop();
        }

    }
}
