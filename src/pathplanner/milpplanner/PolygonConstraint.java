package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import java.awt.Shape;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;


public class PolygonConstraint implements ObstacleConstraint{
    
    public final Obstacle2DB region;
    
    public PolygonConstraint(Obstacle2DB region){
        this.region = region;
    }

    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, CPLEXSolverConfig config, List<IloIntVar> slackVars)
            throws IloException {
        int largeNum = 99999;
        List<Pos2D> vertices = region.getVertices();
        Helper helper = new Helper(cplex);
        IloIntVar[] slack = cplex.intVarArray(region.getVertices().size(), 0, 1);
        IloConstraint cons = null;
        for(int i = 0; i < vertices.size(); i++){
            Pos2D first = vertices.get(i);
            Pos2D second = vertices.get((i + 1) % vertices.size());
            Pos2D delta = second.minus(first);
            
            IloConstraint newCons;
            double buffer;
            double mult = 1;

            if(delta.x == 0){
                if(config.ignoreVehicleSize){
                    buffer = 0;
                }else{
                    buffer = scenario.vehicle.size;
                }
                if(delta.y <= 0) mult = -1;
                
                
                if(config.useIndicatorConstraints){
                    newCons = cplex.le(mult * first.x + buffer, cplex.prod(mult, vars.posX[t]));
                }else{
                    newCons = cplex.le(cplex.sum(mult * first.x + buffer, cplex.prod(slack[i], -largeNum)), cplex.prod(mult, vars.posX[t]));
                }
                
                // CORNER SKIP
                if(config.preventCornerCutting && t > 0){
                    if(config.useIndicatorConstraints){
                        newCons = cplex.and(newCons,
                                cplex.le(mult * first.x + buffer, cplex.prod(mult, vars.posX[t-1])));
                    }else{
                        newCons =  cplex.and(newCons,
                                cplex.le(cplex.sum(mult * first.x + buffer, cplex.prod(slack[i], -largeNum)), cplex.prod(mult, vars.posX[t-1])));
                    }

                }
//                
                

            }else{
                double a = delta.y / delta.x;
                double b = first.y - a * first.x;
                boolean above = (delta.x < 0);
                if(config.ignoreVehicleSize){
                    buffer = 0;
                }else{
                    double alpha = Math.atan(-1/a);
                    buffer = Math.abs(scenario.vehicle.size / Math.sin(alpha));
                }
                if(!above) mult = -1;
                
                if(config.useIndicatorConstraints){
                    newCons = cplex.le(mult*b + buffer, cplex.diff(cplex.prod(mult, vars.posY[t]), cplex.prod(mult*a, vars.posX[t])));
                }else{
                    newCons = cplex.le(cplex.sum(mult*b + buffer, cplex.prod(slack[i], -largeNum)), 
                            cplex.diff(cplex.prod(mult, vars.posY[t]), cplex.prod(mult*a, vars.posX[t])));
                }
                

                
                // CORNER SKIP
                if(config.preventCornerCutting && t > 0){
                    if(config.useIndicatorConstraints){
                        newCons = cplex.and(newCons,
                                cplex.le(mult*b + buffer, cplex.diff(cplex.prod(mult, vars.posY[t-1]), cplex.prod(mult*a, vars.posX[t-1]))));
                    }else{
                        newCons = cplex.and(newCons,
                                cplex.le(cplex.sum(mult*b + buffer, cplex.prod(slack[i], -largeNum)), 
                                cplex.diff(cplex.prod(mult, vars.posY[t]), cplex.prod(mult*a, vars.posX[t]))));
                    }

                }
                

            }

            if(config.useIndicatorConstraints){
                newCons = cplex.ifThen(helper.isFalse(slack[i]), newCons);
            }
            if(cons == null){
                cons = newCons;
            }else{
                cons = cplex.and(cons, newCons);
            }
            
            
            
        }
     
        cons = cplex.and(cons, cplex.le(cplex.sum(slack), vertices.size() - 1  ));
        
        slackVars.addAll(Arrays.asList(slack));
        return cons;
        
        

    }
    
    public static PolygonConstraint fromRegion(Obstacle2DB region){
        return new PolygonConstraint(region);
    }
    
    public Shape getShape(){
        return region.shape;
    }

}
