package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import pathplanner.common.Scenario;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex) throws IloException;

}
