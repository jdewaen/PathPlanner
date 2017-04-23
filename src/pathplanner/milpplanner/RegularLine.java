package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;

import java.util.List;

import pathplanner.common.Scenario;


public class RegularLine extends Line {
    public final double a;
    public final double b;
    public final boolean above;
    
    
    public RegularLine(double a, double b, boolean above){
        this.a = a;
        this.b = b;
        this.above = above;
    }


    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t,  Scenario scenario, IloCplex cplex, boolean ignoreSize, List<IloIntVar> slackVars) throws IloException {
        
        double diff = 0;
        
//        if (ignoreSize){
//        	diff = 0;
//        }else{
//            double alpha = Math.PI / 2 - Math.atan(-1/a);
//            diff = Math.abs(scenario.vehicle.size / Math.cos(alpha));
//            }
        try{
        if(above){
            return cplex.le(b + diff, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }else{
            return cplex.ge(b - diff, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }
        }catch(Exception e){
            System.out.println("bla");
            throw e;
        }
    }


    @Override
    public IloConstraint preventSkipping(IloCplex cplex, List<IloIntVar> last,
            List<IloIntVar> current) {
        return null;
    }
    
    @Override
    public IloConstraint preventCornerCutting(IloCplex cplex, List<IloIntVar> last,
            List<IloIntVar> current) {
        return null;
    }
}
