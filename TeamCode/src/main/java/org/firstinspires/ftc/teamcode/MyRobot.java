package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.jumpypants.murphy.RobotContext;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.X_arm;
import org.firstinspires.ftc.teamcode.subSystems.Z_arm;
import org.firstinspires.ftc.teamcode.subSystems.Claw;

public class MyRobot extends RobotContext {
    public final MecanumDrive drive;
    public final X_arm xarm;
    public final Z_arm zarm;
    public final Claw claw;

    public MyRobot(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                   MecanumDrive drive, X_arm xarm, Z_arm zarm, Claw claw) {
        super(telemetry, gamepad1, gamepad2);
        this.drive = drive;
        this.xarm = xarm;
        this.zarm = zarm;
        this.claw = claw;
    }
}

