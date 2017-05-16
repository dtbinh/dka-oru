package dka.timelinePlaning.optimizatio;

import java.util.LinkedHashSet;
import java.util.Set;


import aima.core.agent.Action;
import aima.core.search.framework.ActionsFunction;
import aima.core.search.framework.ResultFunction;

public class SequencingFunctionFactory {
	
	private static ActionsFunction _actionsFunction = null;
	private static ResultFunction _resultFunction = null;

	public static ActionsFunction getIActionsFunction() {
		if (null == _actionsFunction) {
			_actionsFunction = new MapActionFunction();
		}
		return _actionsFunction;
	}
	
	
	public static ResultFunction getResultFunction() {
		if (null == _resultFunction) {
			_resultFunction = new MapResultFunction();
		}
		return _resultFunction;
	}
	
	
	private static class MapActionFunction implements ActionsFunction {

		@Override
		public Set<Action> actions(Object s) {
			Set<Action> actions = new LinkedHashSet<Action>();						
			actions.add(Map.SEQUENCE_CHANGE);
			return actions;
		}
	}
	
	private static class MapResultFunction implements ResultFunction{

		@Override
		public Object result(Object s, Action a) {
			
			Map m = (Map) s;
			if(Map.SEQUENCE_CHANGE.equals(a)){
				Map newMap = new Map(m);
				newMap.changeSequence();
				return newMap;
			}
			return m;
		}	
	}

	
}
