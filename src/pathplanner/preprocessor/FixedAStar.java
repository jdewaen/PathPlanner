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


public class FixedAStar extends GridSearchAlgorithm{

    
    public FixedAStar(Scenario scenario){
        this(scenario, scenario.world);
    }
    
    public FixedAStar(Scenario scenario, World2D world){
        super(scenario, world);
    }
    
    public FixedAStar(){
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
                    SearchNode finalNode = new SearchNode(current, goal, current.cost
                            + goal.distanceFrom(current.pos));
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

//    public LinkedList<Node> solve(double gridSize){
//        PriorityQueue<Node> queue = new PriorityQueue<Node>();
//        Set<Pos2D> alreadyDone = new HashSet<Pos2D>();
//        queue.add(new Node(null, scenario.startPos, 0));
//        alreadyDone.add(scenario.startPos);
//        while(queue.size() != 0){
//            Node current = queue.poll();
//            Set<Node> neighbors = generateNeighbors(current, gridSize, alreadyDone);
//            
//            for(Node node : neighbors){
//                if(node.pos.fuzzyEquals(scenario.goal, gridSize * 1.5)){
//                    Node finalNode = new Node(node, scenario.goal, node.cost + scenario.goal.distanceFrom(node.pos));
//                    return finalNode.getPath();
//                }
//            }
//            
//            queue.addAll(neighbors);
//        }
//        
//        return null;
//    }

    private Set<SearchNode> generateNeighbors(SearchNode current, double gridSize, Map<Pos2D, Double> currentBest){
        Set<SearchNode> result = new HashSet<SearchNode>();
        for(int x = -1; x <=1 ; x++){
            for(int y = -1; y <=1 ; y++){
                if(x == 0 && y == 0) continue;

                Pos2D newPos = new Pos2D(current.pos.x + x*gridSize, current.pos.y + y*gridSize);   

                if(!isPossiblePosition(newPos)) continue;
                if(!scenario.world.isInside(newPos)) continue;

                double cost = current.cost;
                if(Math.abs(x) + Math.abs(y) == 2){
                    cost += SQRT2 * gridSize;
                }else{
                    cost += gridSize;
                }
                SearchNode newNode = new SearchNode(current, newPos, cost);
                
                if(currentBest.containsKey(newPos) && currentBest.get(newPos) <= newNode.cost) continue;
                currentBest.put(newPos, newNode.cost);
                result.add(newNode);
            }    
        }
        return result;
    }
    
    private Set<SearchNode> generateNeighbors(SearchNode current, double gridSize, Set<Pos2D> alreadyDone){
        Set<SearchNode> result = new HashSet<SearchNode>();
        for(int x = -1; x <=1 ; x++){
            for(int y = -1; y <=1 ; y++){
                if(x == 0 && y == 0) continue;

                Pos2D newPos = new Pos2D(current.pos.x + x*gridSize, current.pos.y + y*gridSize);   
                if(isFuzzyInSet(alreadyDone, newPos)) continue;
                alreadyDone.add(newPos);
                if(!isPossiblePosition(newPos)) continue;
                if(!scenario.world.isInside(newPos)) continue;

                double cost = current.cost;
                if(Math.abs(x) + Math.abs(y) == 2){
                    cost += SQRT2 * gridSize;
                }else{
                    cost += gridSize;
                }
                SearchNode newNode = new SearchNode(current, newPos, cost);
                
                result.add(newNode);
            }    
        }
        return result;
    }

    private boolean isFuzzyInSet(Set<Pos2D> alreadyDone, Pos2D pos){
        for(Pos2D current : alreadyDone){
            if(pos.fuzzyEquals(current, 0.001)) return true;
        }
        return false;
    }

    private boolean isPossiblePosition(Pos2D pos){
        for(Obstacle2DB region : scenario.world.getObstacles()){
            if(region.fuzzyContains(pos, scenario.vehicle.size)) return false;
        }
        return true;
    }

    @Override
    public FixedAStar buildAlgo(Scenario scenario, World2D world) {
        return new FixedAStar(scenario, world);
    }




}
