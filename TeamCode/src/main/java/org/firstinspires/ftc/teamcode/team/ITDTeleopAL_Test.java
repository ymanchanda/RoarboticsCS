package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.auto.ITDBaseLACH;
import org.firstinspires.ftc.teamcode.team.states.ITDClawArmStateMachine;


/*
 * This {@code class} acts as the driver-controlled program for FTC team 16598 for the Into The Deep
 * challenge. By extending {@code ITDTeleopRobotCHALLC}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks      -> Mecanum drive
 *          Left-Bumper                 -> Decrease robot speed .7x
 *          Right-Bumper                -> Normal robot speed 1x
 *      Lift:
 *          Y-Button                    -> Extend lift to "Out" position
 *          B-Button                    -> Extend lift to "In" position
 *          A-Button                    -> Retract lift to starting position
 *      Arm:
 *          Left_trigger                -> Turn arm to "PickUp" position
 *          Right_trigger               -> Turn arm to "Drop" position
 *          X-Button (pressed)          -> Turn arm to Starting position
 *
 *  User 2:
 *      Claw:
 *          Left-trigger                ->
 *          Right-trigger               ->
 *          A-button (pressed)          ->
 *          Y-button (pressed)          ->
 *      ClawArm:
 *          Dpad-up                     ->
 *          Dpad-down                   ->
 *
 */
@TeleOp(name = "ITD TeleOp AL", group = "Main")
public class ITDTeleopAL_Test extends ITDTeleopRobotCHALLC {

    private double currentTime = 0; // keep track of current time
    //these are based on LiftTest
    private static final double Out = 17d;
    private static final double In = 10d;
    private static final double High = 22d;
    private static final double PickUp = 4d;
    private static final double Drop = 6d;


    private Pose2d poseEstimate;

    @Override
    public void init(){
        drive = new ITDBaseLACH(hardwareMap, true);
        super.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        poseEstimate = drive.getPoseEstimate();
//---------------------------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

        //Arm
        while(getEnhancedGamepad1().isRightBumperJustPressed()){
            drive.robot.getITDArmSubsystem().setSetpoint(+1);
        }
        if (getEnhancedGamepad1().isDpadDownJustPressed()) {
            drive.robot.getITDArmSubsystem().setSetpoint(PickUp);
        }
        if (getEnhancedGamepad1().getRight_trigger() > 0) {
            drive.robot.getITDArmSubsystem().setSetpoint(Drop);
        }
        if (getEnhancedGamepad1().isxJustPressed()) {
            drive.robot.getITDArmSubsystem().retract();
        }
        if (getEnhancedGamepad1().isDpadDownJustPressed()) {
            drive.robot.getITDArmSubsystem().setSetpoint(-7);
        }

        //Lift
        if(getEnhancedGamepad1().isDpadRightJustPressed()){
            drive.robot.getITDLiftSubsystem().extend(High);
        }
        if(getEnhancedGamepad1().isyJustPressed()){
            drive.robot.getITDLiftSubsystem().extend(Out);
        }
        if(getEnhancedGamepad1().isbJustPressed()){
            drive.robot.getITDLiftSubsystem().extend(In);
        }
        if(getEnhancedGamepad1().isaJustPressed()){
            drive.robot.getITDLiftSubsystem().retract();
            }


        telemetry.addData("Lift State: ", drive.robot.getITDLiftSubsystem().getStateMachine().getState());
        telemetry.addData("Arm State: ", drive.robot.getITDArmSubsystem().getStateMachine().getState());


        updateTelemetry(telemetry);
        currentTime = getRuntime();
    }

}
