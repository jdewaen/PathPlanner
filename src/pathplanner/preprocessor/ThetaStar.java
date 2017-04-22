package pathplanner.preprocessor;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.World2D;


public class ThetaStar extends GridSearchAlgorithm{

    public ThetaStar(Scenario scenario, World2D world){
        super(scenario, world);
    }
    
    public ThetaStar(Scenario scenario){
        this(scenario, scenario.world);
    }
    
    public ThetaStar(){
    }
    
    @Override
    public PathNode solve(double gridSize) {
        return solve(gridSize, scenario.startPos);
    }
    
    @Override
    public PathNode solve(double gridSize, Pos2D start){
        return solve(gridSize, start, 0);
    }
    @Override
    public PathNode solve(double gridSize, Pos2D start, double startCost){
        Map<Pos2D, PathNode> resultMap = new HashMap<Pos2D, PathNode>();
        resultMap.put(scenario.goal, null);
        solve(gridSize, start, resultMap, startCost);
        return resultMap.get(scenario.goal);
    }
    
    @Override
    public void solve(double gridSize, Pos2D start,
            Map<Pos2D, PathNode> result) {
        solve(gridSize, start, result, 0);
    }

    @Override
    public void solve(double gridSize, Pos2D start,
            Map<Pos2D, PathNode> result, double startCost) {
        
        Set<Pos2D> goalsTodo = new HashSet<Pos2D>(result.keySet());
        PriorityQueue<SearchNode> queue = new PriorityQueue<SearchNode>();
        Map<Pos2D, Double> currentBest = new HashMap<Pos2D, Double>();
        Set<Pos2D> toRemove = new HashSet<Pos2D>();

        currentBest.put(start, startCost);
        queue.add(new SearchNode(null, start, startCost));
        
        while (queue.size() != 0 && !goalsTodo.isEmpty()) {
            SearchNode current = queue.poll();
            toRemove.clear();

            for (Pos2D goal : goalsTodo) {
                if (current.pos.fuzzyEquals(goal, gridSize * 1.5)) {
                    SearchNode finalNode;
                    if(current.parent != null && lineOfSight(current.parent.pos, goal)){
                        double cost = current.parent.cost + current.parent.pos.distanceFrom(goal);
                        finalNode = new SearchNode(current.parent, goal, cost);
                    }else{
                        finalNode = new SearchNode(current, goal, current.cost
                            + goal.distanceFrom(current.pos));
                    }
                    result.put(goal, finalNode.getPath());
                    toRemove.add(goal);
                }
            }
            goalsTodo.removeAll(toRemove);

            Set<SearchNode> neighbors = generateNeighbors(current, gridSize,
                    currentBest);

            queue.addAll(neighbors);
        }
    }

    private Set<SearchNode> generateNeighbors(SearchNode current, double gridSize, Map<Pos2D, Double> currentBest){
        Set<SearchNode> result = new HashSet<SearchNode>();
        for(int x = -1; x <=1 ; x++){
            for(int y = -1; y <=1 ; y++){
                if(x == 0 && y == 0) continue;
                if(Math.abs(x) + Math.abs(y) == 2) continue;

                Pos2D newPos = new Pos2D(current.pos.x + x*gridSize, current.pos.y + y*gridSize);   

                if(!isPossiblePosition(newPos, gridSize, current.pos)) continue;
                if(!world.isInside(newPos)) continue;

                SearchNode newNode;
                if(current.parent != null && lineOfSight(current.parent.pos, newPos)){
                    double cost = current.parent.cost + current.parent.pos.distanceFrom(newPos);
                    newNode = new SearchNode(current.parent, newPos, cost);
                }else{
                    double cost = current.cost;
                    if(Math.abs(x) + Math.abs(y) == 2){
                        cost += SQRT2 * gridSize;
                    }else{
                        cost += gridSize;
                    }
                    newNode = new SearchNode(current, newPos, cost);
                }
                if(currentBest.containsKey(newPos) && currentBest.get(newPos) <= newNode.cost) continue;
                currentBest.put(newPos, newNode.cost);
                result.add(newNode);
            }    
        }
        return result;
    }

    private boolean isPossiblePosition(Pos2D pos, double gridSize, Pos2D last){
        for(Obstacle2DB region : world.getObstacles()){
            if(region.fuzzyContains(pos, gridSize / 2)){
                if(region.intersects(pos, last, scenario.vehicle.size)) return false;
            }
        }
        return true;
    }
    
    private boolean lineOfSight(Pos2D pos1, Pos2D pos2){
        for(Obstacle2DB region : scenario.world.getObstacles()){
            if(region.intersects(pos1, pos2, 0)) return false;
        }
        return true;
    }
    
    @Override
    public ThetaStar buildAlgo(Scenario scenario, World2D world) {
        return new ThetaStar(scenario, world);
    }

}
