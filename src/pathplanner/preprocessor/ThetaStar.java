package pathplanner.preprocessor;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
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
    public LinkedList<Node> solve(double gridSize) {
        return solve(gridSize, scenario.startPos);
    }
    
    @Override
    public LinkedList<Node> solve(double gridSize, Pos2D start){
        Map<Pos2D, LinkedList<Node>> resultMap = new HashMap<Pos2D, LinkedList<Node>>();
        resultMap.put(scenario.goal, new LinkedList<Node>());
        solve(gridSize, start, resultMap);
        return resultMap.get(scenario.goal);
    }

    @Override
    public void solve(double gridSize, Pos2D start,
            Map<Pos2D, LinkedList<Node>> result) {
        
        Set<Pos2D> goalsTodo = new HashSet<Pos2D>(result.keySet());
        PriorityQueue<Node> queue = new PriorityQueue<Node>();
        Map<Pos2D, Double> currentBest = new HashMap<Pos2D, Double>();
        Set<Pos2D> toRemove = new HashSet<Pos2D>();

        currentBest.put(start, (double) 0);
        queue.add(new Node(null, start, 0));
        
        while (queue.size() != 0 && !goalsTodo.isEmpty()) {
            Node current = queue.poll();
            toRemove.clear();

            for (Pos2D goal : goalsTodo) {
                if (current.pos.fuzzyEquals(goal, gridSize * 1.5)) {
                    Node finalNode;
                    if(current.parent != null && lineOfSight(current.parent.pos, goal)){
                        double cost = current.parent.cost + current.parent.pos.distanceFrom(goal);
                        finalNode = new Node(current.parent, goal, cost);
                    }else{
                        finalNode = new Node(current, goal, current.cost
                            + goal.distanceFrom(current.pos));
                    }
                    result.put(goal, finalNode.getPath());
                    toRemove.add(goal);
                }
            }
            goalsTodo.removeAll(toRemove);

            Set<Node> neighbors = generateNeighbors(current, gridSize,
                    currentBest);

            queue.addAll(neighbors);
        }
    }

    private Set<Node> generateNeighbors(Node current, double gridSize, Map<Pos2D, Double> currentBest){
        Set<Node> result = new HashSet<Node>();
        for(int x = -1; x <=1 ; x++){
            for(int y = -1; y <=1 ; y++){
                if(x == 0 && y == 0) continue;

                Pos2D newPos = new Pos2D(current.pos.x + x*gridSize, current.pos.y + y*gridSize);   

                if(!isPossiblePosition(newPos)) continue;
                if(!world.isInside(newPos)) continue;

                Node newNode;
                if(current.parent != null && lineOfSight(current.parent.pos, newPos)){
                    double cost = current.parent.cost + current.parent.pos.distanceFrom(newPos);
                    newNode = new Node(current.parent, newPos, cost);
                }else{
                    double cost = current.cost;
                    if(Math.abs(x) + Math.abs(y) == 2){
                        cost += SQRT2 * gridSize;
                    }else{
                        cost += gridSize;
                    }
                    newNode = new Node(current, newPos, cost);
                }
                if(currentBest.containsKey(newPos) && currentBest.get(newPos) <= newNode.cost) continue;
                currentBest.put(newPos, newNode.cost);
                result.add(newNode);
            }    
        }
        return result;
    }

    private boolean isPossiblePosition(Pos2D pos){
        for(Obstacle2DB region : world.getObstacles()){
            if(region.fuzzyContains(pos, scenario.vehicle.size)) return false;
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
