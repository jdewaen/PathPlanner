package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.List;

import pathplanner.common.Vector2D;
import pathplanner.common.Vehicle;


public class PathSegment {
    
    public final PathNode start;
    public final PathNode end;
    public double goalVel = Double.NaN;
    
    public PathSegment(PathNode start, PathNode end){
        this.start = start;
        this.end = end;
    }
    
    public static List<Vector2D> toPositions(List<PathSegment> segments){
        List<Vector2D> result = new ArrayList<Vector2D>();
        if(segments == null) return result;
        for(PathSegment segment : segments){
            result.add(segment.start.pos);
            result.add(segment.end.pos);
        }
                
        return result;
    }
    
    public List<Vector2D> toIndividualPositions(){
        List<Vector2D> result = new ArrayList<Vector2D>();
        PathNode current = end;
        while(current.distance >= start.distance){
            result.add(current.pos);
            if(current.parent == null) break;
            current = current.parent;
        }
        return result;
    }
    
    
    public double estimateTimeNeeded(Vehicle vehicle, double multiplier){
//        double turnAngle = Math.acos(getStartVector().dotProduct(getFinishVector()));
//        if(turnAngle == 0) return Math.max(3 * getDistance()/vehicle.maxSpeed, minimum); // 2 should be enough, add buffer
        
        double maxSegmentSpeed = Math.min(Math.sqrt(getDistance() * vehicle.acceleration / 2), vehicle.maxSpeed);
        
        double accTime = vehicle.getAccTime(maxSegmentSpeed);
        double accDist = vehicle.getAccDist(maxSegmentSpeed);
        
        int accTimes;
        if(isStraight()){
            // no corner: 1 acceleration and break
            accTimes = 2;
        }else{
            // corner: 2 accelerations and breaks
            accTimes = 4;
        }
        double distanceLeft = getDistance() - accTimes*accDist;
        double timeNeeded = accTimes*accTime;
        if(distanceLeft > 0) timeNeeded += distanceLeft / maxSegmentSpeed;
        timeNeeded *= multiplier;
//        System.out.println("OLD: " + 3*getDistance()/vehicle.maxSpeed + " NEW: " + timeNeeded);
        return timeNeeded;
    }
    
    public boolean isStraight(){
        return start.child == end;
    }
    
    public double getDistance(){
        return end.distance - start.distance;
    }
    
    public Vector2D getFinishVector(){
        PathNode prev = end.getParent();
        PathNode next = end.getChild();
        if(next == null) next = end;

        Vector2D result = next.pos.minus(prev.pos).normalize();
        return result;
    }
    
    public Vector2D getStartVector(){
        PathNode prev = start.getParent();
        PathNode next = start.getChild();
        if(prev == null) prev = start;

        Vector2D result = next.pos.minus(prev.pos).normalize();
        return result;
    }
    
    public double getStartExpansion(){
        return start.distanceFrom(start.child);
    }
    public double getEndExpansion(){
        return end.distanceFrom(end.parent);
    }

}
