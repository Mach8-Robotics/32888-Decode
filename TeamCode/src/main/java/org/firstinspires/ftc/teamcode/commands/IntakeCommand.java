package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private double power = 1;
    private long time;
    Timing.Timer timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);
        public IntakeCommand(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
            time = 2000;
        }
        public IntakeCommand(IntakeSubsystem intakeSubsystem,long time) {
            this.intakeSubsystem = intakeSubsystem;
            this.time = time;
            addRequirements(intakeSubsystem);
        }
    @Override
    public void initialize(){
            timer.start();
            intakeSubsystem.intake(power);
    }
}
