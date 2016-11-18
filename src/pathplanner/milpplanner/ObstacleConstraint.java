package pathplanner.milpplanner;

import pathplanner.common.Scenario2D;
import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario2D scenario, IloCplex cplex) throws IloException;

}
