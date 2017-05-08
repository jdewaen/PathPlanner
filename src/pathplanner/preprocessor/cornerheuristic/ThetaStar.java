package pathplanner.preprocessor.cornerheuristic;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
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


public class ThetaStar extends CornerHeuristic{
    
    public final ThetaStarConfig config;

    public ThetaStar(Scenario scenario, World2D world, ThetaStarConfig config){
        super(scenario, world);
        this.config = config;
    }
    
    public ThetaStar(Scenario scenario, ThetaStarConfig config){
        this(scenario, scenario.world, config);
    }
    
    public ThetaStar(Scenario scenario) {
        this(scenario, ThetaStarConfig.DEFAULT);
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
                    SearchNode finalNode;
                    if(current.parent != null && lineOfSight(current.parent.pos, goal)){
                        double distance = current.parent.distance + current.parent.pos.distanceFrom(goal);
                        finalNode = new SearchNode(current.parent, goal, distance);
                    }else{
                        finalNode = new SearchNode(current, goal, current.distance
                            + goal.distanceFrom(current.pos));
                    }
                    return finalNode.getPath();
                }
            

            Set<SearchNode> neighbors = generateNeighbors(current, currentBest, goal);

            queue.addAll(neighbors);
        }
        return null;
    }

    private Set<SearchNode> generateNeighbors(SearchNode current, Map<Pos2D, Double> currentBest, Pos2D goal){
        Set<SearchNode> result = new HashSet<SearchNode>();
        for(int x = -1; x <=1 ; x++){
            for(int y = -1; y <=1 ; y++){
                if(x == 0 && y == 0) continue;
                if(Math.abs(x) + Math.abs(y) == 2) continue;

                Pos2D newPos = new Pos2D(current.pos.x + x*config.gridSize, current.pos.y + y*config.gridSize);   

                if(!isPossiblePosition(newPos, config.gridSize, current.pos)) continue;
                if(!world.isInside(newPos)) continue;

                SearchNode newNode;
                double heuristic = newPos.distanceFrom(goal);
                if(current.parent != null && lineOfSight(current.parent.pos, newPos)){
                    double distance = current.parent.distance + current.parent.pos.distanceFrom(newPos);
                    newNode = new SearchNode(current.parent, newPos, distance, heuristic);
                }else{
                    double distance = current.distance;
                    if(Math.abs(x) + Math.abs(y) == 2){
                        distance += SQRT2 * config.gridSize;
                    }else{
                        distance += config.gridSize;
                    }
                    newNode = new SearchNode(current, newPos, distance, heuristic);
                }
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
    
    private boolean lineOfSight(Pos2D pos1, Pos2D pos2){
        double x = Math.min(pos1.x, pos2.x);
        double y = Math.min(pos1.y, pos2.y);
        double w = Math.abs(pos1.x - pos2.x);
        double h = Math.abs(pos1.y - pos2.y);
        Rectangle2D boundingBox = new Rectangle2D.Double(x, y, w, h);
        for(Obstacle2DB region : scenario.world.getObstaclesForPositions(pos1, pos2)){
            if(!region.boundingBoxOverlaps(boundingBox)) continue;
            if(region.intersects(pos1, pos2, 0)) return false;
        }
        return true;
    }
    
    
    public List<CornerEvent> generateEvents(PathNode path){
        double maxDeltaCost = config.tolerance * scenario.vehicle.getAccDist();
        ArrayList<PathNode> list = path.toArrayList();
        List<CornerEvent> result = new ArrayList<CornerEvent>();
        int i = 0;
        
        while(i < list.size() - 1){
            PathNode current = list.get(i);

            PathNode lastNodeOfCorner = current;
            PathNode currentCornerNode = list.get(i+1);
            int turnDirection = lastNodeOfCorner.getTurnDirection();
            if(turnDirection == 0){
                System.out.println("NO TURN AT: " + lastNodeOfCorner.pos.toPrettyString());
                i++;
                continue;
            }
            i++;
            while(i < list.size() - 1){
                currentCornerNode = list.get(i);
                PathNode nextNode = list.get(i+1);
                if(currentCornerNode.distance - lastNodeOfCorner.distance > maxDeltaCost){
                    break;
                }
                if(currentCornerNode.getTurnDirection() != turnDirection){
                    break; //TODO: check distance,  don't break if really close (half? quarter?)
                    // DIRECTION CHANGED!!! MAKE EVENT AND KEEP GOING HERE
                }else{
                    lastNodeOfCorner = currentCornerNode;
                    currentCornerNode = nextNode;
                    i++;
                    continue;
                    // SAME DIRECTION: UPDATE NODE
                }

            }
            
            // Corner is fully expanded
            CornerEvent event = new CornerEvent(current, lastNodeOfCorner);
            current = currentCornerNode;
            result.add(event);
        }
        return result;
        
    }
}
