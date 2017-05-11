package pathplanner.preprocessor.cornerheuristic;

import java.util.HashSet;
import java.util.Set;

import pathplanner.common.Pos2D;
import pathplanner.preprocessor.PathNode;


public class SearchNode implements Comparable<SearchNode>{
    Set<SearchNode> children = new HashSet<SearchNode>();
    public final Pos2D pos;
    public SearchNode parent;
    public final double distance;
    public final double heuristic;
    
    public SearchNode(SearchNode parent, Pos2D pos, double distance, double heuristic){
        this.pos = pos;
        this.parent = parent;
        this.distance = distance;
        this.heuristic = heuristic;
        if(parent != null){
            parent.children.add(this);
        }
    }
    public SearchNode(SearchNode parent, Pos2D pos, double distance){
        this(parent, pos, distance, 0);
    }
    
    public double distanceFrom(SearchNode other){
        return other.pos.distanceFrom(pos);
    }
    
    public PathNode getPath(){
        PathNode result = new PathNode(null, pos, distance, PathNode.PathNodeType.ESSENTIAL);
        SearchNode last = this;
        SearchNode current = parent;
        while(current != null){
            result.insertBefore(new PathNode(null, current.pos, current.distance, PathNode.PathNodeType.ESSENTIAL));
            result = result.getParent();
            Set<SearchNode> currentChildren = new HashSet<SearchNode>(current.children);
            for(SearchNode child : currentChildren){
                if(child == last) {
                    last = current;
                    current = current.parent;
                    break;
                }
            }
        }
        return result;
    }

    @Override
    public int compareTo(SearchNode other) {
        return Double.compare(distance + heuristic, other.distance + other.heuristic);
    }

    
//    public Node getChild(){
//        for(Node child : children){
//            return child;
//        }
//        return null;
//    }
    
   
    
    
    public String toString(){
        return pos.toPrettyString() + ":" + String.valueOf(distance);
    }
}
