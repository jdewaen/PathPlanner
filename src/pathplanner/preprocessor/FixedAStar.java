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
        return solve(gridSize, start, scenario.goal);
    }

    @Override
    public PathNode solve(double gridSize, Pos2D start, Pos2D goal) {
        
        PriorityQueue<SearchNode> queue = new PriorityQueue<SearchNode>();
        Map<Pos2D, Double> currentBest = new HashMap<Pos2D, Double>();

        currentBest.put(start, (double) 0);
        queue.add(new SearchNode(null, start, 0));
        
        while (queue.size() != 0) {
            SearchNode current = queue.poll();

                if (current.pos.fuzzyEquals(goal, gridSize * 1.5)) {
                    SearchNode finalNode = new SearchNode(current, goal, current.distance
                            + goal.distanceFrom(current.pos));
                    return finalNode.getPath();
                }

            Set<SearchNode> neighbors = generateNeighbors(current, gridSize,
                    currentBest, goal);

            queue.addAll(neighbors);
        }
        
        return null;
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

    private Set<SearchNode> generateNeighbors(SearchNode current, double gridSize, Map<Pos2D, Double> currentBest, Pos2D goal){
        Set<SearchNode> result = new HashSet<SearchNode>();
        for(int x = -1; x <=1 ; x++){
            for(int y = -1; y <=1 ; y++){
                if(x == 0 && y == 0) continue;

                Pos2D newPos = new Pos2D(current.pos.x + x*gridSize, current.pos.y + y*gridSize);   

                if(!isPossiblePosition(newPos, gridSize, current.pos)) continue;
                if(!scenario.world.isInside(newPos)) continue;

                double distance = current.distance;
                double heuristic = newPos.distanceFrom(goal);
                if(Math.abs(x) + Math.abs(y) == 2){
                    distance += SQRT2 * gridSize;
                }else{
                    distance += gridSize;
                }
                SearchNode newNode = new SearchNode(current, newPos, distance, heuristic);
                
                if(currentBest.containsKey(newPos) && currentBest.get(newPos) <= newNode.distance) continue;
                currentBest.put(newPos, newNode.distance);
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

    private boolean isPossiblePosition(Pos2D pos, double gridSize, Pos2D last){
        for(Obstacle2DB region : scenario.world.getObstaclesForPositions(pos, last)){
            if(region.fuzzyContains(pos, gridSize / 2)) return false;
            if(region.intersects(pos, last, scenario.vehicle.size)) return false;

        }
        return true;
    }

    @Override
    public FixedAStar buildAlgo(Scenario scenario, World2D world) {
        return new FixedAStar(scenario, world);
    }




}
