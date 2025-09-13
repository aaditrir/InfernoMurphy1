package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.states.StateMachine;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.X_arm;
import org.firstinspires.ftc.teamcode.subSystems.Z_arm;
import org.firstinspires.ftc.teamcode.subSystems.Claw;

@TeleOp(name="Murphy Example Tele-Op", group="Linear OpMode")
public class MainTeleOp extends OpMode {

    private StateMachine stateMachine;
    private MyRobot robot;

    @Override
    public void init() {
        // Initialize subsystems
        Z_arm zarm = new Z_arm(hardwareMap);
        X_arm xarm = new X_arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        com.arcrobotics.ftclib.drivebase.MecanumDrive drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                hardwareMap.get(Motor.class, "frontLeft"),
                hardwareMap.get(Motor.class, "backLeft"),
                hardwareMap.get(Motor.class, "frontRight"),
                hardwareMap.get(Motor.class, "backRight")
        );

        robot = new MyRobot(telemetry, gamepad1, gamepad2, drive, xarm, zarm, claw);

        stateMachine = new StateMachine(new IntakingState(robot), robot);
    }

    @Override
    public void loop() {
        stateMachine.step();

        robot.telemetry.addData("X Arm Pos", robot.xarm.getCurrentPosition());
        robot.telemetry.addData("Z Arm Pos", robot.zarm.getCurrentPosition());
        robot.telemetry.update();
    }
}

