package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.List;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;


public class CheckpointGenerator {
    
    Scenario scenario;
    List<Obstacle2DB> obstacles;
    List<List<Pos2D>> vertices;
    
    public CheckpointGenerator(Scenario scenario){
        this.scenario = scenario;
        generateVertices();
    }
    
    private void generateVertices(){
        obstacles = new ArrayList<Obstacle2DB>();
        vertices = new ArrayList<List<Pos2D>>();
        List<Obstacle2DB> regions = scenario.world.getObstacles();
        for(int i = 0; i < regions.size(); i++){
            Obstacle2DB region = regions.get(i);
            obstacles.add(region);
            vertices.add(region.getVertices());
        }
    }
    
    public List<CornerEvent> generateCornerEvents(PathNode path, double gridSize, double tolerance){
        long startTime = scenario.stats.startTimer();
        List<CornerEvent> corners = CornerEvent.generateEvents(path, scenario.vehicle.getAccDist() * tolerance);
        scenario.stats.cornerTime = scenario.stats.stopTimer(startTime);
        return corners;
    }
    
    public List<PathSegment> generateFromPath(PathNode path, double gridSize, List<CornerEvent> corners, double margin, double maxTime){
        long startTime = scenario.stats.startTimer();

           
        List<PathSegment> result = expandCornerEvents(corners, path, scenario.vehicle.getAccDist() * margin, scenario.vehicle.maxSpeed * maxTime);
        scenario.stats.segmentTime = scenario.stats.stopTimer(startTime);

        return result;

        
    };
    
    private PathNode expandForwards(PathNode start, double distance){
        double goal = start.distance + distance;
        PathNode current = start;
        while(current.getChild() != null && current.distance < goal){
            current = current.getChild();
        }
        if(current.distance < goal){
            return current;
        }else{
            PathNode parent = current.parent;
            Pos2D diff = current.pos.minus(parent.pos);
            diff = diff.normalize();
            diff = diff.multiply(goal - parent.distance);
            Pos2D newPos = parent.pos.plus(diff);
            PathNode inter = new PathNode(parent, newPos, goal);
            parent.insertAfter(inter);
            return inter;
        }
        
        
    }
    
    private PathNode expandBackwards(PathNode start, double distance){
        double goal = start.distance - distance;
        PathNode current = start;
        PathNode last = null;
        
        while(current.parent != null && current.distance > goal){
            last = current;
            current = current.parent;          
        }
        
        if(current.distance > goal){
            return current;
        }else{
            PathNode child = last;
            Pos2D diff = child.pos.minus(current.pos);
            diff = diff.normalize();
            diff = diff.multiply(goal - current.distance);
            Pos2D newPos = current.pos.plus(diff);
            PathNode inter = new PathNode(current, newPos, goal);
            current.insertAfter(inter);
            return inter;
        }
        
    }
    
    private List<PathSegment> expandCornerEvents(List<CornerEvent> events, PathNode path, double expansionDist, double maxLength){
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        PathNode lastSegmentEnd = path.getFirst();
        
        if(events.isEmpty()){
        	result.addAll(segmentize(lastSegmentEnd, path.getLast(), maxLength));
        	return result;
        }
        
        // For each corner corner
        boolean noCatchUp = false;
        for(int i = 0; i < events.size(); i++){
            CornerEvent currentEvent = events.get(i);
            
            
            /// *** CATCH UP ***
            //find the desired start node for the corner
            PathNode cornerStart = expandBackwards(currentEvent.start, expansionDist);
            //catch up from last segment end if needed
            if(!noCatchUp && cornerStart.isAfter(lastSegmentEnd)){
                result.addAll(segmentize(lastSegmentEnd, cornerStart, maxLength));
                lastSegmentEnd = cornerStart;
            }else{
                cornerStart.remove();
            }
            
            
            /// *** IF LAST CORNER ***
            if(i == events.size() - 1){
                // For the last corner: expand backwards from end to find desired last segment transition
                PathNode endSegmentStart = expandBackwards(path.getLast(), expansionDist*2);
                
                // If this is already before end of second-to-last corner, just construct from that end to finish;
                if(endSegmentStart == lastSegmentEnd || !endSegmentStart.isAfter(lastSegmentEnd)){
                    result.addAll(segmentize(lastSegmentEnd, path.getLast(), maxLength));
                    endSegmentStart.remove();
                }else{
                // Else: take the desired expansion and give the last corner the remaining hole
                    result.addAll(segmentize(lastSegmentEnd, endSegmentStart, maxLength));
                    result.addAll(segmentize(endSegmentStart, path.getLast(), maxLength));
                }
                break;
            }
                  
            
            
          /// *** IF NOT LAST CORNER ***
            PathNode nextEventStart = events.get(i + 1).start;
            // If there is plenty of space between end of this corner and the start of the next
            if(nextEventStart.pathDistanceFrom(currentEvent.end) > 3 * expansionDist){
                noCatchUp = false; // The next iteration will need to catch up
                
                //expand forwards
                PathNode cornerEnd = expandForwards(currentEvent.end, expansionDist);
        
                // Add corner (possibly segmented) if the end of the last segment is before the desired end for this corner
                if(lastSegmentEnd.isBefore(cornerEnd)){
                    result.addAll(segmentize(lastSegmentEnd, cornerEnd, maxLength));
                    lastSegmentEnd = cornerEnd;
                }else{
                    cornerEnd.remove();
                }

                
            }else{
                // There is not plenty of room between the current and next corner
                noCatchUp = true; // We will end the segment in the middle, so the next iteration does not need to catch up
                
                // expand find the middle between the corners (even if their boundaries overlap)
                double diff = Math.abs(nextEventStart.distance - currentEvent.end.distance);
                PathNode cornerEnd = expandForwards(currentEvent.end, diff/2);
                
                // calculate the maximum save approach speed
                double approachSpeed = scenario.vehicle.getMaxSpeedFromDistance(diff/2);
                                
                result.addAll(segmentize(lastSegmentEnd, cornerEnd, maxLength, approachSpeed));
                lastSegmentEnd = cornerEnd;
            }
        }
        
        return result;
        
    }
   
   

    private List<PathSegment> segmentize(PathNode start, PathNode end, double maxLength, double finalApproachSpeed){
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        PathNode current = start;
        
        int segments = (int) Math.ceil((end.distance - current.distance) / maxLength);
        double targetLength = (end.distance - current.distance) / segments;
        
        for(int i = 0; i < segments - 1; i++){
            PathNode segmentLast = expandForwards(current, targetLength);
            result.add(new PathSegment(current, segmentLast));
            current = segmentLast;
        }
        
        PathSegment last = new PathSegment(current, end);
        last.goalVel = finalApproachSpeed;
        result.add(last);

        return result; 
    }
    
    private List<PathSegment> segmentize(PathNode start, PathNode end, double maxLength){
        return segmentize(start, end, maxLength, Double.NaN);
    }

}
