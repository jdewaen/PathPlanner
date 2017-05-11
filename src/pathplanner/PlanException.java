package pathplanner;

import pathplanner.common.Solution;


public class PlanException extends Exception {

    /**
     * 
     */
    private static final long serialVersionUID = -854260723197824785L;
    
    public final Solution sol;
    public final Exception parent;
    
    public PlanException(Solution sol, Exception parent){
        this.sol = sol;
        this.parent = parent;
    }

}
