package org.firstinspires.ftc.teamcode;

import android.media.CamcorderProfile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous (name = "SHARK SUB AUTONOMOUS", group = "Autonomous")
class SharkSubAutonomous extends LinearOpMode {
    // declare variables

    FtcDashboard dashboard;
    //declare OpMose members.
    ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx left1 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx left2= null;
    private DcMotorEx right2 = null;

    public static PIDFCoefficients DrivetrainPID = new PIDFCoefficients(25,0.05,1.25,0);
    PIDFCoefficients pidOrig,currentPID;

    //everything in inches, like the field
    final double wheelCircum = (2 * Math.PI * 2);
    final double countsPerRev = 28 * 20;
    public double greatest_dist = 0;
    final double EcPerInch = countsPerRev / wheelCircum;

    @Override
    public void runOpMode() {

        left1 = hardwareMap.get(DcMotorEx.class, "leftFront");
        right1 = hardwareMap.get(DcMotorEx.class, "leftBack");
        left2 = hardwareMap.get(DcMotorEx.class, "rightFront");
        right2 = hardwareMap.get(DcMotorEx.class, "rightBack");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pidOrig = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        left1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        left2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        currentPID = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoders();

        waitForStart(); //autonomous action starts from here

        /*
        Steps to take:
        1. Camera sensor to detect signal pattern AND navigation picture
        2. Move forward 2 tiles
        3. Turn left 45º (depending on navigation picture) towards medium junction so robot is parallel to audience
        4. Score preload onto medium junction (4 points)
        5. Turn left 45º
        6. Move into parking position
            Location 1: move forwards 1 tile
            Location 2: stay as it is
            Location 3: move backwards 1 tile
         */


        driveForward(72, 0.5);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void driveForward(double distance, double power){

        int target = (int)(distance * countsPerRev);
        double error = 0;
        double newTarget = getCurrentPosition() + target;

        //The next 8 lines use built-in PID.
        left1.setTargetPosition(target);
        left2.setTargetPosition(target);
        right1.setTargetPosition(target);
        right2.setTargetPosition(target);

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && left1.isBusy() && left2.isBusy() && right1.isBusy() && right2.isBusy() && left1.getCurrentPosition() < newTarget){
            error = target - getCurrentPosition();
            print(target, dashboard.getTelemetry());
            left1.setPower(power);
            left2.setPower(power);
            right1.setPower(power);
            right2.setPower(power);
        }
        stopMotors();

        while (opModeIsActive() || error > 0){
            error = target - getCurrentPosition();
            print(target, dashboard.getTelemetry());
        }

    }

    public void turnLeft(double power){
        left1.setPower(-power);
        left2.setPower(-power);
        right1.setPower(power);
        right2.setPower(power);
    }

    public void stopMotors(){
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
    }

    public double getCurrentPosition(){
        return (left1.getCurrentPosition() + left2.getCurrentPosition() + right1.getCurrentPosition() + right2.getCurrentPosition()) / 4; // Math.abs returns the absolute value of an argument
    }

    public void resetEncoders(){
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void print(double target, Telemetry dashboardTelemetry){

        double dist = getCurrentPosition()/countsPerRev;

        if(dist>greatest_dist) {
            greatest_dist = dist;
        }

        dashboardTelemetry.addData("Distance", dist);
        dashboardTelemetry.addData("Peak", greatest_dist);
        dashboardTelemetry.addData("Error", (target- getCurrentPosition())/countsPerRev);
        dashboardTelemetry.addData("Original PID coef", pidOrig);
        dashboardTelemetry.addData("Current PID coef", currentPID);
        dashboardTelemetry.addData("Tolerance",   left2.getTargetPositionTolerance());
        dashboardTelemetry.addData("Peak time", time);

        dashboardTelemetry.update();
    }

}
