package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
@Config

public class intakeTest extends OpMode {
    AnalogInput turretEncoder;
    DcMotorEx flyWheel;
    Intake intake;
    DcMotor turret;


    double intakeLposition;
    double intakeRposition;
    public static double offsetValueEncoder;
    public static double offsetValueTurret;

    public void init(){
        intake = new Intake(hardwareMap);
        turret= hardwareMap.get(DcMotorEx.class, "turret");
        turretEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop(){
        double MelonboticsEncoderPosition = turretEncoder.getVoltage() / 3.2 * 360;
        telemetry.addData("MelonboticsEncoderPosition:", MelonboticsEncoderPosition);


        double offset = offsetValueEncoder + 360;
        double mboffsetPosition = ((turretEncoder.getVoltage() / 3.2 * 120 + offset) % 120)-60;
        telemetry.addData("MelonboticsEncoderPosition with Offset:", mboffsetPosition);

        double turretEncoderPosition= (turret.getCurrentPosition() * 360) / 1400;
        intake.shootBalls();
        flyWheel.setPower(0);

/*
        if (gamepad1.left_bumper){
            intakeLposition+=0.01;
        }
        if (gamepad1.right_bumper){
            intakeLposition-=0.01;
        }
        telemetry.addData("intakeLposition", intakeLposition);

        if (gamepad1.a){
            intakeRposition+=0.01;
        }
        if (gamepad1.b){
            intakeRposition-=0.01;
        }

 */
        telemetry.addData("intakeRposition", intakeRposition);
        telemetry.addData("Velocity", flyWheel.getVelocity());
        telemetry.addData("turret position", turretEncoderPosition );

    }
}
