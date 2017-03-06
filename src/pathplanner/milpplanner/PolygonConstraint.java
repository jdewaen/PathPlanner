package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;

import java.awt.Shape;
import java.util.Arrays;
import java.util.List;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;


public class PolygonConstraint implements ObstacleConstraint{
    
    public final Obstacle2DB region;
    
    protected PolygonConstraint(Obstacle2DB region){
        this.region = region;
    }

    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize, List<IloIntVar> slackVars)
            throws IloException {
        int largeNum = 99999;
        List<Pos2D> vertices = region.getVertices();

//        double buffer;
//        
//        if (ignoreSize){
//        	buffer = 0;
//        }else{
//        	buffer = scenario.vehicle.size;	
//        }
//        
        IloIntVar[] slack = cplex.intVarArray(region.getVertices().size(), 0, 1);
        IloConstraint cons = null;
        for(int i = 0; i < vertices.size(); i++){
            Pos2D first = vertices.get(i);
            Pos2D second = vertices.get((i + 1) % vertices.size());
            Pos2D delta = second.minus(first);
            
            IloConstraint newCons;
            double buffer;

            if(delta.x == 0){
                if(ignoreSize){
                    buffer = 0;
                }else{
                    buffer = scenario.vehicle.size;
                }
                if(delta.y > 0){
                    newCons = cplex.le(cplex.sum(first.x + buffer, cplex.prod(slack[i], -largeNum)), vars.posX[t]);
                }else{
                    newCons = cplex.ge(cplex.sum(first.x - buffer, cplex.prod(slack[i], largeNum)), vars.posX[t]);

                }
            }else{
                double a = delta.y / delta.x;
                double b = first.y - a * first.x;
                boolean above = (delta.x < 0);
                if(ignoreSize){
                    buffer = 0;
                }else{
                    double alpha = Math.PI / 2 - Math.atan(-1/a);
                    buffer = Math.abs(scenario.vehicle.size / Math.cos(alpha));
                }
                if(above){
                    newCons = cplex.le(cplex.sum(b + buffer, cplex.prod(slack[i], -largeNum)), cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
                }else{
                    newCons = cplex.ge(cplex.sum(b - buffer, cplex.prod(slack[i], largeNum)), cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
                }
            }

            if(cons == null){
                cons = newCons;
            }else{
                cons = cplex.and(cons, newCons);
            }
            
//            activeSet.add(new RegularLine(a, b, (delta.x > 0)));
        }
        cons = cplex.and(cons, cplex.addLe(cplex.sum(slack), vertices.size() - 1));
//        return cplex.ge(vars.posX[t], 0);
        
        slackVars.addAll(Arrays.asList(slack));
        return cons;
//        IloConstraint cons = cplex.le(cplex.sum(vars.posX[t], buffer), cplex.sum(region.bottomRightCorner.x, cplex.prod(largeNum, slack[0])));
//        cons = cplex.and(cons, cplex.le(cplex.negative(cplex.sum(vars.posX[t], -buffer)), cplex.sum(-region.topLeftCorner.x, cplex.prod(largeNum, slack[1]))));
//        cons = cplex.and(cons, cplex.le(cplex.sum(vars.posY[t], buffer), cplex.sum(region.bottomRightCorner.y, cplex.prod(largeNum, slack[2]))));
//        cons = cplex.and(cons, cplex.le(cplex.negative(cplex.sum(vars.posY[t], -buffer)), cplex.sum(-region.topLeftCorner.y, cplex.prod(largeNum, slack[3]))));
//        cons = cplex.and(cons, cplex.addLe(cplex.sum(slack), 3));
//        return cons;
        
//        for(int i = 0; i < activeRegion.size(); i++){
//            Pos2D first = activeRegion.get(i);
//            Pos2D second = activeRegion.get((i + 1) % activeRegion.size());
//            Pos2D delta = second.minus(first);
//            
//            double a = delta.y / delta.x;
//            double b = first.y - a * first.x;
//            activeSet.add(new RegularLine(a, b, (delta.x > 0)));
//        }
//        
//        
//        double diff;
//        
//        if (ignoreSize){
//            diff = 0;
//        }else{
//            double alpha = Math.PI / 2 - Math.atan(-1/a);
//            diff = scenario.vehicle.size / Math.cos(alpha);
//            }
//        if(above){
//            return cplex.le(cplex.sum(b - diff, cplex.prod(vars.fin[t], LARGE_NUM)), cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
//        }else{
//            return cplex.ge(cplex.sum(b + diff, cplex.prod(vars.fin[t], LARGE_NUM)), cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
//        }
        
        
        

    }
    
    public static PolygonConstraint fromRegion(Obstacle2DB region){
        return new PolygonConstraint(region);
    }
    
    public Shape getShape(){
        return region.shape;
    }

}
