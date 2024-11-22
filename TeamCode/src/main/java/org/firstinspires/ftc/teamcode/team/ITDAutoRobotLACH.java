package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubServo;
import org.firstinspires.ftc.teamcode.team.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team.subsystems.ITDExpansionHubsLACH;
import org.firstinspires.ftc.teamcode.team.subsystems.ITDLiftSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ITDArmSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ITDClawSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ITDClawArmSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.RobotStateEstimator;


/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel     -> LF
 *         Back Left Wheel      -> LR
 *         Front Right Wheel    -> RF
 *         Back Right Wheel     -> RR
 *
 *     Arm
 *         Right Arm motor      -> RightArm
 *         Left Arm motor       -> LeftArm
 *
 *     Lift
 *          Lift motor          -> Lift
 *
 *     Hang
 *          Hang motor          -> Hang
 *
 * Servo naming convention:
 *     Claw
 *         Claw   servo         -> Claw
 *
 *     ClawArm
 *         ClawArm servo        -> ClawArm
 *
 *     Lock
 *          Lock servo          -> Lock
 *
 */
public class ITDAutoRobotLACH {
    private TimeProfiler matchRuntime;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    private ITDExpansionHubsLACH itdExpansionHubsLACH;
    private ITDLiftSubsystem itdLiftSubsystem;
    private ITDArmSubsystem itdArmSubsystem;
    private ITDClawSubsystem itdClawSubsystem;
    private ITDClawArmSubsystem itdClawArmSubsystem;
    private RevMotor[] motors;
    private RevServo[] servos;


    public void init(HardwareMap hardwareMap) {
//        setExpansionHubs(new ExpansionHubs(this,
//                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
//                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
//        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Lift")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 1.503937), //38.2mm diameter
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Arm Right")), false, true, true, false, Motor.GOBILDA_117_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Arm Left")), false, true, true, false, Motor.GOBILDA_117_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093)
        });

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("Claw"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("ClawArm"))),
        });

        setITDLiftSubsystem(new ITDLiftSubsystem(getMotors()[0]));
        setITDArmSubsystem(new ITDArmSubsystem(getMotors()[1], getMotors()[2]));
        setITDClawSubsystem(new ITDClawSubsystem(getServos()[0]));
        setITDClawArmSubsystem(new ITDClawArmSubsystem(getServos()[1]));
        setMatchRuntime(new TimeProfiler(false));
    }
    public RevMotor[] getMotors() {
        return motors;
    }

    public void setMotors(RevMotor[] motors) {
        this.motors = motors;
    }

    public RevServo[] getServos() {
        return servos;
    }

    public void setServos(RevServo[] servos) {
        this.servos = servos;
    }

    public ITDExpansionHubsLACH getExpansionHubs() {
        return itdExpansionHubsLACH;
    }

    public void setITDExpansionHubsLACH(ITDExpansionHubsLACH itExpansionHubsLACH) {
        this.itdExpansionHubsLACH = itExpansionHubsLACH;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public ITDArmSubsystem getITDArmSubsystem() {
        return itdArmSubsystem;
    }

    public void setITDArmSubsystem(ITDArmSubsystem itdArmSubsystem){
        this.itdArmSubsystem = itdArmSubsystem;
    }
    public ITDLiftSubsystem getITDLiftSubsystem() {
        return itdLiftSubsystem;
    }

    public void setITDLiftSubsystem(ITDLiftSubsystem itdLiftSubsystem){
        this.itdLiftSubsystem = itdLiftSubsystem;
    }

    public ITDClawArmSubsystem getITDClawArmSubsystem() {
        return itdClawArmSubsystem;
    }

    public void setITDClawArmSubsystem(ITDClawArmSubsystem itdClawArmSubsystem){
        this.itdClawArmSubsystem = itdClawArmSubsystem;
    }
    public ITDClawSubsystem getITDClawSubsystem() {
        return itdClawSubsystem;
    }

    public void setITDClawSubsystem(ITDClawSubsystem itdClawSubsystem){
        this.itdClawSubsystem = itdClawSubsystem;
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}

