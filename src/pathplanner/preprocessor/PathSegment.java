package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;


public class PathSegment {
    
    public final Set<Region2D> obstacles;
    public final Node start;
    public final Node end;
    
    public PathSegment(Node start, Node end, Set<Region2D> obstacles){
        this.obstacles = obstacles;
        this.start = start;
        this.end = end;
    }
    
    public PathSegment(Node start, Node end){
        this(start, end, new HashSet<Region2D>());
    }
    
    public static List<Pos2D> toPositions(List<PathSegment> segments){
        List<Pos2D> result = new ArrayList<Pos2D>();
        for(PathSegment segment : segments){
            result.add(segment.start.pos);
        }
        
        result.add(segments.get(segments.size()-1).end.pos);
        
        return result;
    }

}
