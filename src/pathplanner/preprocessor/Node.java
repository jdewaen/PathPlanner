package pathplanner.preprocessor;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;

import pathplanner.common.Pos2D;


public class Node implements Comparable<Node>{
    Set<Node> children = new HashSet<Node>();
    public final Pos2D pos;
    public final Node parent;
    public final double cost;
    
    public Node(Node parent, Pos2D pos, double cost){
        this.pos = pos;
        this.parent = parent;
        this.cost = cost;
        if(parent != null){
            parent.children.add(this);
        }
    }
    
    public double distanceFrom(Node other){
        return other.pos.distanceFrom(pos);
    }
    
    public LinkedList<Node> getPath(){
        LinkedList<Node> result = new LinkedList<Node>();
        result.addFirst(this);
        Node last = this;
        Node current = parent;
        while(current != null){
            result.addFirst(current);
            Set<Node> currentChildren = new HashSet<Node>(current.children);
            for(Node child : currentChildren){
                if(child != last) current.children.remove(child);
            }
            last = current;
            current = current.parent;
        }
        return result;
    }

    @Override
    public int compareTo(Node other) {
        return Double.compare(cost, other.cost);
    }

}
