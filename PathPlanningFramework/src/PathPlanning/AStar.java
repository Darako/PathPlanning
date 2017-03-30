package PathPlanning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AStar extends SearchMethod {

    private List<Node> open;
    private List<Node> closed;
	
	public AStar(Map dem, Node init, Node end, short heuristic, boolean withZ, boolean withC) {
		super("AStar", dem, init, end, heuristic, withZ, withC, false);		
		open = new ArrayList<>();	
		closed = new ArrayList<>();
		for(int j = 0; j < map.get_nrows(); j++) {
	        for(int i = 0; i < map.get_ncols(); i++) {
	            map.get_node(i, j).setParent(null);
//	            if(dem.get_tcost(i, j) > 1.00) 
//	            	map.get_node(i, j).OBS = 1.00f;
//	            map.get_node(i, j).setZ(dem.get_tcost(i, j));
	        }
	    }		
		init.setG(0);
		init.setH(get_h(init, init, end));
		init.setF(init.getG()+init.getH());
		init.setParent(init);
		open.add(init);
	}
	

	@Override
	protected void clear_internal_data() {
		open.clear(); // References in open list lost in DEM clean (new nodes)
		for(int j = 0; j < map.get_nrows(); j++) {
	        for(int i = 0; i < map.get_ncols(); i++) {
	            map.get_node(i, j).setParent(null);
	        }
	    }		
	}

	@Override
	public boolean search() {	
		if(!check_valid_data())
			return false;		    
		Node nodoActual = start;
		ArrayList<Node> sucesores = new ArrayList<Node>();	  
		ArrayList<Node> path = new ArrayList<Node>();		
		while(!open.isEmpty()){
			Collections.sort(open);
			nodoActual = open.remove(0);
			if(nodoActual.equals(goal)){
				path = get_path(nodoActual); 
				print_path(path);
				return true;
			}
			closed.add(nodoActual);
			sucesores = map.get_succesors(nodoActual);
			for(int i = 0; i < sucesores.size(); i++){
				Node sucesor = sucesores.get(i);
//				if(!sucesor.isObstacle()){
				if((map.cost[sucesor.getX()][sucesor.getY()]) =< (map.MAX_COST)){
					if(!closed.contains(sucesor)){
						if(!open.contains(sucesor)){
							sucesor.setG(Float.MAX_VALUE);
							sucesor.setParent(null);
						}
						UpdateVertex(nodoActual, sucesor);
					}
				}				
			}			
		}	
		return true;
	}
	
	
	public void UpdateVertex(Node nodoActual, Node nodoSucesor){
		float gActual, gSucesor;
		float hActualSucesor;
		gActual = nodoActual.getG();
		gSucesor = nodoSucesor.getG();
		hActualSucesor = get_h(nodoActual, nodoSucesor, goal);
		if(gActual+hActualSucesor < gSucesor){
			gSucesor = gActual+hActualSucesor;
			nodoSucesor.setG(gSucesor);
			nodoSucesor.setParent(nodoActual);
			if(open.contains(nodoSucesor)){
				open.remove(nodoSucesor);
			}
			open.add(nodoSucesor);
		}
		
	}
	
	
	public ArrayList<Node> get_path(Node dest) {
		ArrayList<Node> camino = new ArrayList<Node>();
		Node nodoActual = dest;
		while(!nodoActual.getParent().equals(nodoActual)){
			camino.add(nodoActual);
			nodoActual = nodoActual.getParent();
		}
		camino.add(nodoActual);
		return camino;
    }
	
    /**
     * Print the open list.
     */
    private void print_path(ArrayList<Node> path)
    {   
        System.out.println("Camino :");
        for(int i=0; i < path.size(); i++)
            System.out.println("    " + path.get(i).toString());
    }	
}
