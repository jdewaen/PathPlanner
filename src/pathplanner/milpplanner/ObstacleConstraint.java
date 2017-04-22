package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;

import java.util.List;

import pathplanner.common.Scenario;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize, List<IloIntVar> slackVars) throws IloException;
    
    public IloConstraint preventSkipping(IloCplex cplex, List<IloIntVar> last, List<IloIntVar> current) throws IloException;
    
    public IloConstraint preventCornerCutting(IloCplex cplex, List<IloIntVar> last, List<IloIntVar> current) throws IloException;


}
