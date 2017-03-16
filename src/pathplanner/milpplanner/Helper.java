package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntExpr;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import java.util.Collection;


public final class Helper {
    public IloCplex cplex;
    public Helper(IloCplex cplex){
        this.cplex = cplex;
    }
    
    public IloConstraint iff(IloConstraint a, IloConstraint b) throws IloException{
        IloConstraint r1 = cplex.or(a, cplex.not(b));
        IloConstraint r2 = cplex.or(b, cplex.not(a));
        return cplex.and(r1, r2);
    }

    public IloConstraint isTrue(IloIntExpr a) throws IloException{
        return cplex.eq(1, a);
    }
    
    public IloIntExpr oneIfTrue(IloIntExpr a) throws IloException{
        return a;
    }
    
    public IloIntExpr oneIfFalse(IloIntExpr a) throws IloException{
        return cplex.diff(1, a);
    }
    
    public IloIntExpr sum(IloIntExpr... vars) throws IloException{
        return cplex.sum(vars);
    }
    
    public IloNumExpr sum(IloNumExpr... vars) throws IloException{
        return cplex.sum(vars);
    }
    

    public IloConstraint and(IloConstraint... vars) throws IloException{
        return cplex.and(vars);
    }
    
    public  IloConstraint[] consListtoArray(Collection<IloConstraint> vars) throws IloException{
        IloConstraint[] result = new IloConstraint[vars.size()];
        int i = 0;
        for(IloConstraint cons : vars){
            result[i++] = cons;
        }
        return result;
    }
    
    public  IloNumExpr[] NumExprListtoArray(Collection<IloNumExpr> vars) throws IloException{
        IloNumExpr[] result = new IloConstraint[vars.size()];
        int i = 0;
        for(IloNumExpr cons : vars){
            result[i++] = cons;
        }
        return result;
    }
    
    public  IloIntExpr[] IntExprListtoArray(Collection<IloIntExpr> vars) throws IloException{
        IloIntExpr[] result = new IloConstraint[vars.size()];
        int i = 0;
        for(IloIntExpr cons : vars){
            result[i++] = cons;
        }
        return result;
    }
    public IloConstraint and(Collection<IloConstraint> vars) throws IloException{
        return cplex.and(consListtoArray(vars));
    }
    
    public IloConstraint or(IloConstraint... vars) throws IloException{
        return cplex.or(vars);
    }
    
    public IloConstraint isFalse(IloNumVar a) throws IloException{
        return cplex.not(cplex.eq(1, a));
    }

    public IloConstraint diff(IloNumVar a, IloNumVar b, double val) throws IloException{
        return cplex.le(cplex.abs(cplex.diff(a, b)), val);
    }

    public IloConstraint diff(double a, IloNumVar b, double val) throws IloException{
        return cplex.le(cplex.abs(cplex.diff(a, b)), val);
    }

    public IloConstraint diff(IloNumVar a, double b, double val) throws IloException{
        return cplex.le(cplex.abs(cplex.diff(a, b)), val);
    }
}
