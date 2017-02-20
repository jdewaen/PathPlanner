package pathplanner.common;

import ilog.concert.IloIntVar;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import pathplanner.milpplanner.ObstacleConstraint;

public class Solution implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -5760134721447460262L;
    public final double maxTime;
    public final int timeSteps;
    public Pos2D[] pos;
    public Pos2D[] vel;
    public Pos2D[] absVel;
    public double[] horiThrottle;
    public double[] vertThrottle;
    public boolean[] fin;
    public boolean[] cfin;
    public boolean[] nosol;
    public double[] time;
    public int[] segment;
//    public HashMap<Region2D, Double> checkpoints = new HashMap<Region2D, Double>();
//    public HashMap<Region2D, Double[]> checkpointCounter = new HashMap<Region2D, Double[]>();
//    public HashMap<Region2D, Double[]> checkpointChange = new HashMap<Region2D, Double[]>();
    
    public HashSet<Pos2D> highlightPoints = new HashSet<Pos2D>();
    public List<List<Pos2D>> activeArea;
    public HashSet<Obstacle2DB>[] activeObstacles;
    public Map<Obstacle2DB, Map<Integer, List<Boolean>>> slackVars;

    
    public int score;
    
    @SuppressWarnings("unchecked")
    public Solution(double maxTime, int timeSteps){
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        pos = new Pos2D[timeSteps];
        vel = new Pos2D[timeSteps];
        absVel = new Pos2D[timeSteps];
        horiThrottle = new double[timeSteps];
        vertThrottle = new double[timeSteps];
        fin = new boolean[timeSteps];
        cfin = new boolean[timeSteps];
        time = new double[timeSteps];
        activeArea = new ArrayList<List<Pos2D>>();
        activeObstacles = new HashSet[timeSteps];
        nosol = new boolean[timeSteps];
        score = 0;
        segment = new int[timeSteps];
        
        slackVars = new HashMap<Obstacle2DB, Map<Integer, List<Boolean>>>();
        
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
//        Map<Integer, Integer> startMap = new HashMap<Integer, Integer>();
        int counter = 0;
        double lastTime = 0;
        int currentSegment = 0;
//        startMap.put(currentSegment, counter);
        for(Solution sol :  list){
            lastTime = result.time[counter];
            result.highlightPoints.addAll(sol.highlightPoints);
            if(sol.isEmpty()) continue;
            final int counterFinal = counter;
            for(Entry<Obstacle2DB, Map<Integer, List<Boolean>>> entry: sol.slackVars.entrySet()){
                Map<Integer, List<Boolean>> offsetMap = entry.getValue().entrySet().stream().collect(
                        Collectors.toMap(el -> counterFinal + el.getKey(), el -> el.getValue()));
                if(result.slackVars.containsKey(entry.getKey())){
                    result.slackVars.get(entry.getKey()).putAll(offsetMap);
                }else{
                    result.slackVars.put(entry.getKey(), offsetMap);
                }
            }
            
            for(int i = 0; i <= sol.score; i++){
                result.pos[counter] = sol.pos[i];
                
                result.vel[counter] = sol.vel[i];
                result.absVel[counter] = sol.absVel[i];
                
                result.horiThrottle[counter] = sol.horiThrottle[i];
                result.vertThrottle[counter] = sol.vertThrottle[i];
                
                result.time[counter] = lastTime + sol.time[i];
                
                if(result.activeArea.size() == counter){
                    result.activeArea.add(sol.activeArea.get(i));
                }else if(result.activeArea.size() == counter + 1){
                    result.activeArea.set(counter, sol.activeArea.get(i));
                }else{
                    throw new RuntimeException("bad combination of solution");
                }
                result.activeObstacles[counter] = sol.activeObstacles[i];
//                
                result.nosol[counter] = sol.nosol[i];
                result.segment[counter] = currentSegment;

                 // convert!!!
                
                counter++;
            }
            

            
            result.score += sol.score;
            currentSegment++;
            counter--;
//            startMap.put(currentSegment, counter);
            
        }
    
        return result;
    } 
    
    public boolean isEmpty(){
        return timeSteps == 0;
    }
    
    public static Solution generateEmptySolution(){
        Solution sol = new Solution(1, 1);
        sol.nosol[0] = true;
        return sol;
    }

}