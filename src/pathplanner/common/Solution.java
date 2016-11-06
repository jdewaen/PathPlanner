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
    
    public double[] t1;
    public double[] t2;
    public double[] q1;
    public double[] w1;
    public double score;
    public Solution(double maxTime, int timeSteps){
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        pos = new Pos2D[timeSteps];
        vel = new Pos2D[timeSteps];
        horiThrottle = new double[timeSteps];
        vertThrottle = new double[timeSteps];
        fin = new boolean[timeSteps];
        cfin = new boolean[timeSteps];
        
        t1 = new double[timeSteps];
        t2 = new double[timeSteps];
        q1 = new double[timeSteps];
        w1 = new double[timeSteps];
        
    }

}