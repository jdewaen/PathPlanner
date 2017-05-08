package pathplanner.preprocessor.cornerheuristic;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.World2D;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;


public class FixedAStar extends CornerHeuristic{

    public final AStarConfig config;
    
    public FixedAStar(Scenario scenario, AStarConfig config){
        this(scenario, scenario.world, config);
    }
    
    public FixedAStar(Scenario scenario, World2D world, AStarConfig config){
        super(scenario, world);
        
        this.config = config;
    }
    
    public FixedAStar(Scenario scenario) {
        this(scenario, AStarConfig.DEFAULT);
    }
    
    @Override
    public PathNode solve() {
        return solve(scenario.startPos);
    }

    
    @Override
    public PathNode solve(Pos2D start){
        return solve(start, scenario.goal);
    }

    @Override
    public PathNode solve(Pos2D start, Pos2D goal) {
        
        PriorityQueue<SearchNode> queue = new PriorityQueue<SearchNode>();
        Map<Pos2D, Double> currentBest = new HashMap<Pos2D, Double>();

        currentBest.put(start, (double) 0);
        queue.add(new SearchNode(null, start, 0));
        
        while (queue.size() != 0) {
            SearchNode current = queue.poll();

                if (current.pos.fuzzyEquals(goal, config.gridSize * 1.5)) {
                    SearchNode finalNode = new SearchNode(current, goal, current.distance
                            + goal.distanceFrom(current.pos));
                    return finalNode.getPath();
                }

            Set<SearchNode> neighbors = generateNeighbors(current, config.gridSize,
                    currentBest, goal);

            queue.addAll(neighbors);
        }
        return null;
    }

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

    private boolean isPossiblePosition(Pos2D pos, double gridSize, Pos2D last){
        for(Obstacle2DB region : scenario.world.getObstaclesForPositions(pos, last)){
            if(region.fuzzyContains(pos, gridSize / 2)) return false;
            if(region.intersects(pos, last, scenario.vehicle.size)) return false;

        }
        return true;
    }

    @Override
    public List<CornerEvent> generateEvents(PathNode path) {
        throw new UnsupportedOperationException();
    }




}
