package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.List;

import pathplanner.common.Pos2D;
import pathplanner.common.Vehicle;


public class PathSegment {
    
    public final PathNode start;
    public final PathNode end;
    public double goalVel = Double.NaN;
    
    public PathSegment(PathNode start, PathNode end){
        this.start = start;
        this.end = end;
    }
    
    public static List<Pos2D> toPositions(List<PathSegment> segments){
        List<Pos2D> result = new ArrayList<Pos2D>();
        if(segments == null) return result;
        for(PathSegment segment : segments){
            result.add(segment.start.pos);
            result.add(segment.end.pos);
        }
                
        return result;
    }
    
    public List<Pos2D> toIndividualPositions(){
        List<Pos2D> result = new ArrayList<Pos2D>();
        PathNode current = end;
        while(current.distance >= start.distance){
            result.add(current.pos);
            if(current.parent == null) break;
            current = current.parent;
        }
        return result;
    }
    
    
    public double estimateTimeNeeded(Vehicle vehicle, double minimum){
//        double turnAngle = Math.acos(getStartVector().dotProduct(getFinishVector()));
//        if(turnAngle == 0) return Math.max(3 * getDistance()/vehicle.maxSpeed, minimum); // 2 should be enough, add buffer
        
        double maxSegmentSpeed = Math.min(Math.sqrt(getDistance() * vehicle.acceleration / 2), vehicle.maxSpeed);
        
        double accTime = vehicle.getAccTime(maxSegmentSpeed);
        double accDist = vehicle.getAccDist(maxSegmentSpeed);
        // 2 accelerations and breaks, see what's left?
        double distanceLeft = getDistance() - 4*accDist;
        double timeNeeded = 4*accTime;
        if(distanceLeft > 0) timeNeeded += distanceLeft / maxSegmentSpeed;
        timeNeeded *= 1.5;
//        System.out.println("OLD: " + 3*getDistance()/vehicle.maxSpeed + " NEW: " + timeNeeded);
        return Math.max(timeNeeded, minimum);
    }
    
    public double getDistance(){
        return end.distance - start.distance;
    }
    
    public Pos2D getFinishVector(){
        PathNode prev = end.getParent();
        PathNode next = end.getChild();
        if(next == null) next = end;

        Pos2D result = next.pos.minus(prev.pos).normalize();
        return result;
    }
    
    public Pos2D getStartVector(){
        PathNode prev = start.getParent();
        PathNode next = start.getChild();
        if(prev == null) prev = start;

        Pos2D result = next.pos.minus(prev.pos).normalize();
        return result;
    }

}
