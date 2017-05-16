package dka.timelinePlaning.meta;

import java.time.Duration;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.meta.spatioTemporal.paths.TrajectoryEnvelopeScheduler;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint.Type;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;


public class DKASingleRobotPlanner extends TrajectoryEnvelopeScheduler{
	
	//HashMap<ConstraintNetwork, Vector<AllenIntervalConstraint>> replacedDurations = new HashMap<ConstraintNetwork, Vector<AllenIntervalConstraint>>();
	HashMap<TrajectoryEnvelope, Vector<AllenIntervalConstraint>> storedDurations = new HashMap<TrajectoryEnvelope, Vector<AllenIntervalConstraint>>(); 
	
	public DKASingleRobotPlanner(long origin, long horizon) {
		super(origin, horizon);
		
	}
	
	@Override
	protected void retractResolverSub(ConstraintNetwork metaVariable, ConstraintNetwork metaValue) {		
		TrajectoryEnvelopeSolver solver = (TrajectoryEnvelopeSolver)this.getConstraintSolvers()[0];
		Variable[] vars = metaValue.getVariables();
		TrajectoryEnvelope move = null;
		for (int i = 0; i < vars.length; i++) {
			TrajectoryEnvelope te = (TrajectoryEnvelope)vars[i];
			if(te.getSymbolicVariableActivity().getSymbols()[0].contains("move")){
				move = te;
				break;
			}
		}
		
		
		if(storedDurations.get(move) != null){
			AllenIntervalConstraint duration = storedDurations.get(move).lastElement();
			if(!solver.addConstraint(duration)){
				//System.out.println(storedDurations);
				//System.out.println("---------------------------------------");
				logger.finest("---------------------------------------" );
				logger.finest(" " + duration);
//				/System.out.println(duration);
				logger.finest("---------------------------------------" );
				//System.out.println("---------------------------------------");
			}
			else{
				System.out.println("++++++++++++++++++++++++++++++++++++++++");
				System.out.println(duration);
				System.out.println("++++++++++++++++++++++++++++++++++++++++");
			}
//			storedDurations.get(move).remove(duration);
//			
//			if(replacedDurations.get(metaValue).size() == 1){
//				replacedDurations.remove(metaValue);
//			}
//			else{
				storedDurations.get(move).removeElementAt(storedDurations.get(move).size() - 1);
//			}
			
		}

		
		
//		if(replacedDurations.get(metaValue) != null){
//			AllenIntervalConstraint duration = replacedDurations.get(metaValue).lastElement();
//			if(!solver.addConstraint(duration)){
//				System.out.println("---------------------------------------");
//				System.out.println(duration);
//				System.out.println("---------------------------------------");
//			}
//			else{
//				System.out.println("++++++++++++++++++++++++++++++++++++++++");
//				System.out.println(duration);
//				System.out.println("++++++++++++++++++++++++++++++++++++++++");
//			}
//			if(replacedDurations.get(metaValue).size() == 1){
//				replacedDurations.remove(metaValue);
//			}
//			else{
//				replacedDurations.get(metaValue).removeElementAt(replacedDurations.get(metaValue).size() - 1);
//			}
//			
//		}

	}

	@Override
	protected boolean addResolverSub(ConstraintNetwork metaVariable,
			ConstraintNetwork metaValue) {
				
		TrajectoryEnvelopeSolver solver = (TrajectoryEnvelopeSolver)this.getConstraintSolvers()[0];
		Constraint[] cons = metaValue.getConstraints();
			
		// if there is already Duration constraint on moveTo, remove that, and add the new one

		
		for (int i = 0; i < cons.length; i++) {
			AllenIntervalConstraint c = (AllenIntervalConstraint)cons[i];			
			if(c.getTypes()[0].equals(Type.Duration)){
				TrajectoryEnvelope te = (TrajectoryEnvelope)c.getFrom();
				Constraint[] outcons =  solver.getConstraintNetwork().getOutgoingEdges(te);
				AllenIntervalConstraint oldDuration = null;		
				for (int j = 0; j < outcons.length; j++) {
					if(((AllenIntervalConstraint)outcons[j]).getTypes()[0].equals(Type.Duration)){
						oldDuration = (AllenIntervalConstraint)outcons[j];
						
					}					
				}
				if(oldDuration != null){
					if(storedDurations.get(te) != null){
						storedDurations.get(te).add(oldDuration);
						System.out.println("old --> " + oldDuration);
						System.out.println("new --> " + c);
						solver.removeConstraint(oldDuration);					
					}else{
						Vector<AllenIntervalConstraint> ds = new Vector<AllenIntervalConstraint>();
						ds.add(oldDuration);
						storedDurations.put(te, ds);
						solver.removeConstraint(oldDuration);
						System.out.println("old --> " + oldDuration);
						System.out.println("new --> " + c);
						
					}
					
					
//					if(replacedDurations.get(metaValue) != null){
//						replacedDurations.get(metaValue).add(oldDuration);
//						solver.removeConstraint(oldDuration);					
//					}else{
//						Vector<AllenIntervalConstraint> ds = new Vector<AllenIntervalConstraint>();
//						ds.add(oldDuration);
//						replacedDurations.put(metaValue, ds);
//						solver.removeConstraint(oldDuration);
//						System.out.println("old --> " + oldDuration);
//						System.out.println("new --> " + c);
//						
//					}
					
				}
			}
		}
		return true;

	}

}
