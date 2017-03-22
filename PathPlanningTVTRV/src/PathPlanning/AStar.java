package PathPlanning;

import java.util.ArrayList;
import java.util.List;

public class AStar extends SearchMethod {

    private List<Node> open;
	
	public AStar(Map dem, Node init, Node end, short heuristic, boolean withZ, boolean withC) {
		super("AStar", dem, init, end, heuristic, withZ, withC, false);		
		open = new ArrayList<>();
		for(int i=0; i < map.get_nrows();i++){
			for(int j=0; j < map.get_ncols();j++){
				map.get_node(i, j).setG(Float.MAX_VALUE);
				map.get_node(i, j).setParent(null);
				open.add(map.get_node(i, j));
			}
		}		
	}
	

	@Override
	protected void clear_internal_data() {
		open.clear(); // References in open list lost in DEM clean (new nodes)
		for(int j = 0; j < map.get_nrows(); j++) {
	        for(int i = 0; i < map.get_ncols(); i++) {
	            map.get_node(i, j).setG(Float.MAX_VALUE);
	            map.get_node(i, j).setParent(null);
	            open.add(map.get_node(i, j));
	        }
	    }		
	}

	@Override
	public boolean search() {	
		Node p;	
		ArrayList<Node> closed = new ArrayList<Node>();
		ArrayList<Node> succ = new ArrayList<Node>();		
		while(!open.isEmpty()){
			p = open.remove(open.size());
			if(p == goal){
				return true;
			}
			closed.add(p);
            succ = map.get_succesors(p);
			for (Node t: succ){
				if(!closed.contains(t)){
					if(!open.contains(t)){
						t.setG((Float) null); //pretendia que fuera infinito, pero meh
						t.setParent(null);
					}
					//update vertex?
				}
			}
		}		
		return false;
	}
}