package pathplanner.preprocessor;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Set;

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import pathplanner.common.Scenario;


public class FixedAStar {

    public final Scenario scenario;
    final double SQRT2 = Math.sqrt(2);
    public FixedAStar(Scenario scenario){
        this.scenario = scenario;
    }

    public LinkedList<Node> solve(double gridSize){
        PriorityQueue<Node> queue = new PriorityQueue<Node>();
        Set<Pos2D> alreadyDone = new HashSet<Pos2D>();
        queue.add(new Node(null, scenario.startPos, 0));
        alreadyDone.add(scenario.startPos);
        while(queue.size() != 0){
            Node current = queue.poll();
            Set<Node> neighbors = generateNeighbors(current, gridSize, alreadyDone);
            
            for(Node node : neighbors){
                if(node.pos.fuzzyEquals(scenario.goal, gridSize)){
                    Node finalNode = new Node(node, scenario.goal, node.cost + scenario.goal.distanceFrom(node.pos));
                    return finalNode.getPath();
                }
            }
            
            queue.addAll(neighbors);
        }
        
        return null;
    }

    private Set<Node> generateNeighbors(Node current, double gridSize, Set<Pos2D> alreadyDone){
        Set<Node> result = new HashSet<Node>();
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
                Node newNode = new Node(current, newPos, cost);
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
        for(Region2D region : scenario.world.getRegions()){
            if(region.fuzzyContains(pos, scenario.vehicle.size)) return false; //FIXME: why doesn't it work with the vehicle size?
        }
        return true;
    }

}
