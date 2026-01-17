package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubSystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubSystem driveSubSystem;
    private DoubleSupplier strafe,forward, turn, gryoAngle;
    private boolean fieldCentric;

         public DriveCommand(DriveSubSystem driveSubSystem,
                             DoubleSupplier strafe,
                             DoubleSupplier forward,
                             DoubleSupplier turn,
                             DoubleSupplier gryoAngle,boolean fieldCentric){
            this.driveSubSystem = driveSubSystem;
            this.strafe = strafe;
            this.forward = forward;
            this.turn = turn;
            this.gryoAngle = gryoAngle;
            this.fieldCentric = fieldCentric;
            addRequirements(driveSubSystem);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void execute(){
        driveSubSystem.drive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble(),gryoAngle.getAsDouble(),fieldCentric);
    }
}
