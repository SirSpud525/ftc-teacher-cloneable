package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class PracticeRobot {

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public double nosePwr;
    public Servo leftEye;
    public CRServo nose;
    public IMU imu;

    public void init(){

    }

    //UTILITY

    public void handleMotorPower(){

    }

    private void setDriveMode(final DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ignored) {
        }
    }
    public double yaw(AngleUnit units){
        return imu.getRobotYawPitchRollAngles().getYaw(units);
        /*to be used instead of having the above line everywhere in the code to make offset or sign reversal
        changes easier, as they will be reflected in the entire repo if updated here*/
    }


    //TELEOP


    //AUTO

    public void driveTo(int pos, double pct) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        while ((Math.abs(pos - frontLeft.getCurrentPosition()) + Math.abs(pos - frontRight.getCurrentPosition()) / 2) > 20) {
            // Drive
            int flDistance = pos - frontLeft.getCurrentPosition();
            int frDistance = pos - frontRight.getCurrentPosition();
            int blDistance = pos - backLeft.getCurrentPosition();
            int brDistance = pos - backRight.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            if (Math.abs(pos - frontLeft.getCurrentPosition()) > 150 && Math.abs(pos - frontRight.getCurrentPosition()) > 150) {
                flDrivePower = Math.signum(flDrivePower) * pct;
                frDrivePower = Math.signum(frDrivePower) * pct;
                blDrivePower = Math.signum(blDrivePower) * pct;
                brDrivePower = Math.signum(brDrivePower) * pct;
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);
            }

            // Slowdown
//            flDrivePower = flDrivePower;
//            frDrivePower = frDrivePower;
//            blDrivePower = blDrivePower;
//            brDrivePower = brDrivePower;

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToFromHere(double target) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = yaw(DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 3) {
            currentPosition = yaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

            //removed slowdown stuff

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



}
