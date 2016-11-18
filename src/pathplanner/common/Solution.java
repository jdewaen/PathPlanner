package pathplanner.common;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Solution {
    public final double maxTime;
    public final int timeSteps;
    public Pos2D[] pos;
    public Pos2D[] vel;
    public double[] horiThrottle;
    public double[] vertThrottle;
    public boolean[] fin;
    public boolean[] cfin;
    public double[] time;
    public HashMap<Region2D, Double> checkpoints = new HashMap<Region2D, Double>();
    public HashMap<Region2D, Double[]> checkpointCounter = new HashMap<Region2D, Double[]>();
    public HashMap<Region2D, Double[]> checkpointChange = new HashMap<Region2D, Double[]>();
    public Set<Pos2D> highlightPoints = new HashSet<Pos2D>();

    public int score;
    
    public Solution(double maxTime, int timeSteps){
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        pos = new Pos2D[timeSteps];
        vel = new Pos2D[timeSteps];
        horiThrottle = new double[timeSteps];
        vertThrottle = new double[timeSteps];
        fin = new boolean[timeSteps];
        cfin = new boolean[timeSteps];
        time = new double[timeSteps];
        score = 0;
        
    }
    
    public static Solution combine(List<Solution> list){
        int sum = 0;
        double maxTime = 0;
        for(Solution sol :  list){
            if(!sol.isEmpty()){
                sum += sol.score;
                maxTime += sol.time[sol.score];
            }
        }
        
        Solution result = new Solution(maxTime, sum + 1);         
        
        int counter = 0;
        double lastTime = 0;
        for(Solution sol :  list){
            lastTime = result.time[counter];
            result.highlightPoints.addAll(sol.highlightPoints);
            if(sol.isEmpty()) continue;
            
            for(int i = 0; i <= sol.score; i++){
                result.pos[counter] = sol.pos[i];
                
                result.vel[counter] = sol.vel[i];
                
                result.horiThrottle[counter] = sol.horiThrottle[i];
                result.vertThrottle[counter] = sol.vertThrottle[i];
                
                result.time[counter] = lastTime + sol.time[i];
                                                
                counter++;
            }
            result.score += sol.score;
            counter--;
        }
    
        return result;
    } 
    
    public boolean isEmpty(){
        return timeSteps == 0;
    }

}