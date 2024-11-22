package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class ITDLockStateMachine  extends TimedState<ITDLockStateMachine.State>{

    public ITDLockStateMachine() {
        super(ITDLockStateMachine.State.UNLOCK);
    }

    @Override
    public String getName() {
        return "Lock State Machine";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(10d, TimeUnits.MILLISECONDS);
    }

    public enum State implements Namable {
        UNLOCK("Unlock",0d), LOCK("Lock", 0.4d);

        private final String name;
        private final double Position;

        State(final String name, final double pPosition) {
            this.name          = name;
            this.Position  = pPosition;
        }

        public double getPosition() {
            return Position;
        }

        @Override

        public String getName() {
            return name;
        }
    }
}
