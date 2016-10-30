package pathplanner.common;


public class Solution {
    public final double maxTime;
    public final int timeSteps;
    public Pos2D[] pos;
    public Pos2D[] vel;
    public double[] horiThrottle;
    public double[] vertThrottle;
    public boolean[] fin;
    public boolean[] cfin;
    public Solution(double maxTime, int timeSteps){
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        pos = new Pos2D[timeSteps];
        vel = new Pos2D[timeSteps];
        horiThrottle = new double[timeSteps];
        vertThrottle = new double[timeSteps];
        fin = new boolean[timeSteps];
        cfin = new boolean[timeSteps];
        
    }

}