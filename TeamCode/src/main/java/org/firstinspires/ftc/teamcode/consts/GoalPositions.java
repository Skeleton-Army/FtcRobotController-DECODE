package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.geometry.Pose;

public class GoalPositions {
    public final static Pose BLUE_GOAL_CLOSE = new Pose(2, 137);
    public final static Pose BLUE_GOAL_FAR = new Pose(2, 142);
    public final static Pose RED_GOAL_CLOSE = new Pose(144 - BLUE_GOAL_CLOSE.getX(), BLUE_GOAL_CLOSE.getY());
    public final static Pose RED_GOAL_FAR = new Pose(144 - BLUE_GOAL_FAR.getX(), BLUE_GOAL_FAR.getY());

    public final static Pose BLUE_GOAL_TOP = new Pose(25,144);
    public final static Pose BLUE_GOAL_BUTTOM = new Pose(0,119);
    public final static Pose BLUE_GOAL_CORNER = new Pose(0,144);
    public final static Pose RED_GOAL_TOP = new Pose( 144 - BLUE_GOAL_TOP.getX(),144);
    public final static Pose RED_GOAL_BUTTOM = new Pose(144,119);
    public final static Pose RED_GOAL_CORNER = new Pose(144,144);


}