package org.firstinspires.ftc.teamcode;

/*
 *  Add additional elements to the AutoCommand class if your robot requires them.   Make sure
 *  to update the constructor!!!!
 */

public  class AutoCommand {
        public AutonomousStates.AutoStates state;
        public Drive.MoveType moveType;
        public double         value1;
        public double         value2;
        public double         value3;
        public double         value4;
        public int            timeLimit;

        public AutoCommand (AutonomousStates.AutoStates state, Drive.MoveType moveType, double value1, double value2, double value3, double value4, int timeLimit) {
            this.state     = state;
            this.moveType  = moveType;
            this.value1    = value1;
            this.value2    = value2;
            this.value3    = value3;
            this.value4    = value4;
            this.timeLimit = timeLimit;

        }

}
