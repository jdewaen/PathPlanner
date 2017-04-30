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
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize, List<IloIntVar> slackVars)
            throws IloException {
        int largeNum = 99999;
        List<Pos2D> vertices = region.getVertices();

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
                    double alpha = Math.atan(-1/a);
                    buffer = Math.abs(scenario.vehicle.size / Math.sin(alpha));
                }
                if(above){
                    newCons = cplex.le(b + buffer, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
                    newCons = cplex.ifThen(cplex.eq(slack[i], 0), newCons);
                }else{
                    newCons = cplex.ge(b - buffer, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
                    newCons = cplex.ifThen(cplex.eq(slack[i], 0), newCons);
                }
            }

            if(cons == null){
                cons = newCons;
            }else{
                cons = cplex.and(cons, newCons);
            }
            
        }
//        cons = cplex.addLe(cplex.sum(slack), vertices.size() - 1);
        cons = cplex.and(cons, cplex.addLe(cplex.sum(slack), vertices.size() - 1));
        
        slackVars.addAll(Arrays.asList(slack));
        return cons;
        
        

    }
    
    public static PolygonConstraint fromRegion(Obstacle2DB region){
        return new PolygonConstraint(region);
    }
    
    public Shape getShape(){
        return region.shape;
    }

    
    @Override
    public IloConstraint preventSkipping(IloCplex cplex, List<IloIntVar> last,
            List<IloIntVar> current) throws IloException {
        if(last == null || current == null) return null;
        Helper helper = new Helper(cplex);
        int size = last.size();
        List<IloConstraint> cons = new ArrayList<IloConstraint>();
        for(int i = 0; i < size; i++){
            for(int j = (i + 2) % size; j != (size + i - 1) % size; j = (j+1) % size){
                IloIntVar xt = last.get(i);
                IloIntVar xt1 = current.get(i);
                IloIntVar yt = last.get(j);
                IloIntVar yt1 = current.get(j);
                IloNumVar e = cplex.numVar(0, 1);
                
                // xt - xt1 - yt + yt1 + e<= 3
                cons.add(cplex.le(helper.sum(helper.oneIfTrue(xt),
                                                            helper.oneIfFalse(xt1),
                                                            helper.oneIfFalse(yt),
                                                            helper.oneIfTrue(yt1),
                                                            e
                                                            ),
                                                3));
                
                // all others (t and t + 1) + (1-e) > 0
                IloNumExpr[] sumOfOthers = new IloNumExpr[size - 2];
                int count = 0;
                for(int v = 0; v < size; v++){
                    if(v == i || v == j) continue;
                    sumOfOthers[count++] = cplex.sum(helper.oneIfTrue(last.get(v)), helper.oneIfTrue(current.get(v)));
                }
                
                
                cons.add(cplex.not(cplex.le(
                                cplex.sum(cplex.diff(1, e),
                                        helper.sum(sumOfOthers)
                                        ),
                                0)));
            }
        }
        return helper.and(helper.consListtoArray(cons));
    }
    
    public IloConstraint preventCornerCutting(IloCplex cplex, List<IloIntVar> last,
            List<IloIntVar> current) throws IloException {
        if(last == null || current == null) return null;
        Helper helper = new Helper(cplex);
        int size = last.size();
        List<IloConstraint> cons = new ArrayList<IloConstraint>();
        for(int i = 0; i < size; i++){
            for(int j = -1; j <=1; j+=2){
                IloIntVar xt = last.get(i);
                IloIntVar xt1 = current.get(i);
                IloIntVar yt = last.get((size + i + j)% size);
                IloIntVar yt1 = current.get((size + i + j)% size);
                IloNumVar e = cplex.numVar(0, 1);
                
                // xt - xt1 - yt + yt1 + e<= 3
                cons.add(cplex.le(helper.sum(helper.oneIfTrue(xt),
                                                            helper.oneIfFalse(xt1),
                                                            helper.oneIfFalse(yt),
                                                            helper.oneIfTrue(yt1),
                                                            e
                                                            ),
                                                3));
                
                // sum of all t + (1-e) > 1 
                IloNumExpr[] sumOfAll = new IloNumExpr[size];
                int count = 0;
                for(int v = 0; v < size; v++){
                    sumOfAll[count++] = helper.oneIfTrue(last.get(v));
                }
                cons.add(cplex.not(cplex.le(
                                cplex.sum(cplex.diff(1, e),
                                        helper.sum(sumOfAll)
                                        ),
                                1)));
                
                // sum of all t+1 + (1-e) > 1 
                count = 0;
                for(int v = 0; v < size; v++){
                    sumOfAll[count++] = helper.oneIfTrue(current.get(v));
                }
                cons.add(cplex.not(cplex.le(
                                cplex.sum(cplex.diff(1, e),
                                        helper.sum(sumOfAll)
                                        ),
                                1)));
            }
        }
        return helper.and(helper.consListtoArray(cons));
    }
}
