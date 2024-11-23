package org.firstinspires.ftc.teamcode.Commands.Autos;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.ArmDown;
import org.firstinspires.ftc.teamcode.Commands.ArmUp;
import org.firstinspires.ftc.teamcode.Commands.DriveTo;
import org.firstinspires.ftc.teamcode.Commands.Grab;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.StrafeTo;
import org.firstinspires.ftc.teamcode.Commands.TurnTo;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Auto4Pieces extends SequentialCommandGroup {
    private final Drivetrain dt;
    private final Arm arm;
    private final Intake intake;

    public Auto4Pieces(Drivetrain dt, Arm arm, Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.dt = dt;
        this.arm = arm;
        this.intake = intake;
        addCommands(new DriveTo(20,dt),
                    new WaitCommand(200),
                    new StrafeTo(-50,dt),
                    new WaitCommand(200),
                    new TurnTo(45,dt),
                    new ArmUp(arm),
                    new Outtake(intake),
                    new ArmDown(arm),
                    new TurnTo(-45,dt),
                    new WaitCommand(200),
                    new DriveTo(20,dt),
                    new Grab(intake),
                    new DriveTo(-20,dt),
                    new WaitCommand(200),
                    new TurnTo(45,dt),
                    new ArmUp(arm),
                    new Outtake(intake),
                    new ArmDown(arm),
                    new InstantCommand(()->arm.setArmZero(),arm),
                    new InstantCommand(()->arm.setElevatorZero(),arm),
                    new WaitCommand(15000));
        addRequirements(dt, arm, intake);
    }

}
