/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;


@Autonomous (name = "Bartworst", group = "Linear Opmode")
public class Bartworst extends LinearOpMode {

    double distanceDriven;

    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        setAll(1.0);
        Sleep(250);
        setAll(0);
        Sleep(1000000);

    }

    public void setAll(double power) { //set all motors to same speed, enter speed
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
    }

    public void driveUntil(double DISTANCE) {
        distanceDriven = robot.frontRight.getCurrentPosition();
        while(distanceDriven < DISTANCE) {
            setAll(.5);
        } 
        setAll(0);
    }

    public void moveDownHook() {
        foundation1.setPosition(downPosition);
        foundation2.setPosition(downPosition);
    }

    public void moveUpHook() {
        foundation1.setPosition(upPosition);
        foundation2.setPosition(upPosition);
    }

    public void openClaw() {
        clawLeft.setPosition(clawOpen);
        clawRight.setPosition(clawOpen);
    }

    public void closeClaw() {
        clawLeft.setPosition(clawClosed);
        clawRight.setPosition(clawClosed);
    }

    //ehhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhs
    
    public void turn(double degrees) {
        if( degrees > 0 ) {
            frontRight.setPower();

        }
        robot.frontRight.setPower(.5);
        robot.backRight.setPower(.5);
        robot.backLeft.setPower(-.5);
        robot.frontLeft.setPower(-.5);
    } 


 //ticks = x * inches
 //inches = ticks / x
    public double toInches() {
        return (robot.frontRight.getCurrentPosition / conversion);
    }

    public double toTicks(double inches) {
        return (conversion * inches);
    }

    
}
*/