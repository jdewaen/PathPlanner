package pathplanner.common;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import pathplanner.preprocessor.boundssolver.BoundsSolverDebugData;

public class Solution implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -5760134721447460262L;
    public final double maxTime;
    public final int timeSteps;
    public Vector2D[] pos;
    public Vector2D[] vel;
    public Vector2D[] absVel;
    public Vector2D[] acc;
    public Vector2D[] jerk;
    public boolean[] fin;
    public boolean[] cfin;
    public boolean[] nosol;
    public double[] time;
    public int[] segment;
    
    public List<Vector2D> highlightPoints = new ArrayList<Vector2D>();
    public List<List<Vector2D>> activeArea;
    public HashSet<Obstacle2D>[] activeObstacles;
    public Map<Obstacle2D, Map<Integer, List<Boolean>>> slackVars;
    public List<BoundsSolverDebugData> boundsDebugData;

    
    public int score;
    
    @SuppressWarnings("unchecked")
    public Solution(double maxTime, int timeSteps){
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        pos = new Vector2D[timeSteps];
        vel = new Vector2D[timeSteps];
        absVel = new Vector2D[timeSteps];
        acc = new Vector2D[timeSteps];
        jerk = new Vector2D[timeSteps];
        fin = new boolean[timeSteps];
        cfin = new boolean[timeSteps];
        time = new double[timeSteps];
        activeArea = new ArrayList<List<Vector2D>>();
        boundsDebugData = new ArrayList<BoundsSolverDebugData>();
        activeObstacles = new HashSet[timeSteps];
        nosol = new boolean[timeSteps];
        score = 0;
        segment = new int[timeSteps];
        
        slackVars = new HashMap<Obstacle2D, Map<Integer, List<Boolean>>>();
        
    }
    
    public static Solution combine(List<Solution> list, int overlap){
        Solution first = list.get(0);
        int sum = first.score + 1;
        double maxTime = first.time[first.score];
        for(int i = 1; i < list.size(); i++){
            Solution sol = list.get(i);
            if(!sol.isEmpty()){
                sum += sol.score - overlap + 1;
                maxTime += sol.time[sol.score - overlap + 1];
            }
            
        }
        
        Solution result = new Solution(maxTime, sum);         
        int counter = 0;
        double lastTime = 0;
        int currentSegment = 0;
        for(Solution sol :  list){
            lastTime = result.time[counter];
            if(sol.isEmpty()) continue;
            final int counterFinal = counter;
            for(Entry<Obstacle2D, Map<Integer, List<Boolean>>> entry: sol.slackVars.entrySet()){
                Map<Integer, List<Boolean>> offsetMap = entry.getValue().entrySet().stream().collect(
                        Collectors.toMap(el -> counterFinal + el.getKey(), el -> el.getValue()));
                if(result.slackVars.containsKey(entry.getKey())){
                    result.slackVars.get(entry.getKey()).putAll(offsetMap);
                }else{
                    result.slackVars.put(entry.getKey(), offsetMap);
                }
            }
            
            for(int i = 0; i <= sol.score; i++){
                result.pos[counter+i] = sol.pos[i];
                
                result.vel[counter+i] = sol.vel[i];
                result.absVel[counter+i] = sol.absVel[i];
                
                result.acc[counter+i] = sol.acc[i];
                
                result.jerk[counter+i] = sol.jerk[i];
                
                
                result.time[counter+i] = lastTime + sol.time[i];
                
                if(result.activeArea.size() > counter + i){
                    result.activeArea.set(counter + i, sol.activeArea.get(i));
                    result.boundsDebugData.set(counter + i, sol.boundsDebugData.get(i));

                }else{
                    result.activeArea.add(sol.activeArea.get(i));
                    result.boundsDebugData.add(sol.boundsDebugData.get(i));

                }
                
                result.activeObstacles[counter+i] = sol.activeObstacles[i];              
                result.nosol[counter+i] = sol.nosol[i];
                result.segment[counter+i] = currentSegment;

            }
            
            
            result.score += sol.score - overlap + 1;
            currentSegment++;
            counter += sol.score - overlap + 1;
            result.highlightPoints.add(result.pos[counter]);

            
        }
        result.highlightPoints.remove(result.highlightPoints.size() - 1);
        result.score += overlap - 1;
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