package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class aDrivetrain {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    boolean slowMode;
    boolean holdMode;

    public aDrivetrain(HardwareMap hardwareMap){
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        slowMode = false;

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
    }
    public void driveCA(double right_stick_x, double right_stick_y, double left_stick_x, double left_trigger, double right_trigger){
        double rtx = right_stick_x;
        double rty = right_stick_y;
        double ltx = left_stick_x * 1.1;

        double powerfactor = 1;
        double rotate = -ltx;
        double x = -rtx;
        double y = -rty;
        double fleft = (x + y - rotate) * powerfactor;
        double fright = (-x + y + rotate) * powerfactor;
        double bleft = (-x + y - rotate) * powerfactor;
        double bright = (x + y + rotate) * powerfactor;

        if (slowMode){
            fleft =Math.max(-0.25, Math.min(0.25, fleft));
            fright =Math.max(-0.25, Math.min(0.25, fright));
            bleft =Math.max(-0.25, Math.min(0.25, bleft));
            bright =Math.max(-0.25, Math.min(0.25, bright));

        }
        if (!holdMode) {
            frontLeft.setPower(fleft);
            frontRight.setPower(fright);
            backLeft.setPower(bleft);
            backRight.setPower(bright);
        }

    }

    public void driveIG(double right_stick_x, double right_stick_y, double left_stick_x, double left_trigger, double right_trigger){
        double rtx = right_stick_x;
        double rty = right_stick_y;
        double ltx = left_stick_x * 1.1;

        double powerfactor = 1;
        double rotate = -ltx;
        double x = -rtx;
        double y = -rty;
        double fleft = (x + y - rotate) * powerfactor;
        double fright = (-x + y + rotate) * powerfactor;
        double bleft = (-x + y - rotate) * powerfactor;
        double bright = (x + y + rotate) * powerfactor;

        if (left_trigger > 0.25) {
            frontLeft.setPower(-0.75);
            frontRight.setPower(0.75);
            backLeft.setPower(-0.75);
            backRight.setPower(0.75);
        } else if (right_trigger > 0.25) {
            frontLeft.setPower(0.75);
            frontRight.setPower(-0.75);
            backLeft.setPower(0.75);
            backRight.setPower(-0.75);
        } else {
            frontLeft.setPower(fleft);
            frontRight.setPower(fright);
            backLeft.setPower(bleft);
            backRight.setPower(bright);
        }
    }
    public void setBackLeft(double Power){
        backLeft.setPower(Power);
    }
    public void setBackRight (double power){
        backRight.setPower(power);
    }
    public void setFrontLeft(double power){
        frontLeft.setPower(power);
    }
    public void setFrontRight(double power){
        frontRight.setPower(power);
    }
    public void setSlowmode (boolean slowmode){slowMode= slowmode;}
    public void switchSlowmode (){slowMode= !slowMode;}
    public boolean getSlowmode(){return slowMode;}
    public boolean getHoldMode(){return holdMode;}
    public void switchHoldMode(){
        holdMode = !holdMode;
    }
}
