package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, IloCplex cplex) throws IloException;

}
