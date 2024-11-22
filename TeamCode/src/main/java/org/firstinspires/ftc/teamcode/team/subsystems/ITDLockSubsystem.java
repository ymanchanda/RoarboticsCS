package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team.states.ITDLockStateMachine;

public class ITDLockSubsystem implements ISubsystem<ITDLockStateMachine, ITDLockStateMachine.State>{
    private static ITDLockStateMachine itdLockStateMachine;
    private RevServo LockServo;


    public ITDLockSubsystem(RevServo lockServo){
        setLockStateMachine(new ITDLockStateMachine());
        setLockServo(lockServo);
    }

    @Override
    public ITDLockStateMachine getStateMachine() {
        return itdLockStateMachine;
    }

    @Override
    public ITDLockStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Lock Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getLockServo().setPosition(getState().getPosition());
    }

    public static void setLockStateMachine(ITDLockStateMachine itdLockStateMachine) {
        ITDLockSubsystem.itdLockStateMachine = itdLockStateMachine;
    }

    public RevServo getLockServo() {
        return LockServo;
    }


    public void setLockServo(RevServo lockServo) {
        this.LockServo = lockServo;
    }
}
