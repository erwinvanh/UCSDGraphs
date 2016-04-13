/**
 *
 */
package roadgraph;

import java.io.PrintWriter;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import geography.RoadSegment;
import util.GraphLoader;

/**
 * @author Erwin van Herwijnen, based upon week 2 solution from:  
 *         UCSD MOOC development team
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections of multiple roads.
 * Edges are the roads.
 *
 */
@SuppressWarnings("unused")
public class MapGraph {

	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;
	private static boolean rushHour = false;
	private static boolean useTravelTime = false;

	/** Create a new empty MapGraph
	 *
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	// For us in DEBUGGING.  Print the Nodes in the graph
	public void printNodes()
	{
		System.out.println("****PRINTING NODES ********");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : pointNodeMap.keySet())
		{
			MapNode n = pointNodeMap.get(pt);
			System.out.println(n);
		}
	}

	// For us in DEBUGGING.  Print the Edges in the graph
	public void printEdges()
	{
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge e : edges)
		{
			System.out.println(e);
		}

	}

	/** Add a node corresponding to an intersection
	 *
	 * @param latitude The latitude of the location
	 * @param longitude The longitude of the location
	 * */
	public void addVertex(double latitude, double longitude)
	{
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 *
	 * @param location  The location of the intersection
	 */
	public void addVertex(GeographicPoint location)
	{
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
		}
		else {
			System.out.println("Warning: Node at location " + location +
					" already exists in the graph.");
		}

	}

	/** Add an edge representing a segment of a road.
	 * Precondition: The corresponding Nodes must have already been
	 *     added to the graph.
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 */
	public void addEdge(double lat1, double lon1,
			double lat2, double lon2, String roadName, String roadType)
	{
		// Find the two Nodes associated with this edge.
		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);

	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType) {

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType, double length) {
		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	/** Given a point, return if there is a corresponding MapNode **/
	public boolean isNode(GeographicPoint point)
	{
		return pointNodeMap.containsKey(point);
	}



	// Add an edge when you already know the nodes involved in the edge
	private void addEdge(MapNode n1, MapNode n2, String roadName,
			String roadType,  double length)
	{
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}


	/** Returns the nodes in terms of their geographic locations */
	public Collection<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}

	// get a set of neighbor nodes from a mapnode
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}

	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using Breadth First Search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched)
	{
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();

			// hook for visualization
			nodeSearched.accept(next.getLocation());

			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}

		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);

		return path;
	}

	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */

	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
			MapNode start, MapNode goal)
	{

		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			//System.out.println("Processing " + current.getLocation());
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}


		// add start
		path.addFirst(start.getLocation());

		return path;
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return dijkstra(start, goal, temp);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

		return GenericSearch(start,	goal, nodeSearched, new NodeComparator(), useTravelTime, rushHour);

	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		return GenericSearch(start,	goal, nodeSearched, new AStarNodeComparator(goal), useTravelTime, rushHour);

	}

	// Comparator for Dijkstra (just compare based on distance from start)
	class NodeComparator implements Comparator<MapNode> {

		@Override
		public int compare(MapNode o1, MapNode o2) {

			return o1.getDistance() < o2.getDistance() ? -1 : o1.getDistance() > o2.getDistance() ? 1 : 0;
		}

	}
	// Comparator for AStar (compare based upon distance from start AND estimated
	// distance to goal
	class AStarNodeComparator implements Comparator<MapNode> {

		GeographicPoint goal;

		public AStarNodeComparator(GeographicPoint goal) {

			this.goal = goal;
		}

		@Override
		public int compare(MapNode o1, MapNode o2) {

			double heuristDist1 = o1.getDistance() + o1.getGeoDistanceTo(goal);
			double heuristDist2 = o2.getDistance() + o2.getGeoDistanceTo(goal);
			return heuristDist1 < heuristDist2 ? -1 : heuristDist1 > heuristDist2 ? 1 : 0;
		}
	}	

	/**
	 * Generic search algorithm which can be used for both aStar and Dijkstra since only difference between
	 * aStart and Dijkstra is the Comparator
	 * @param start
	 * @param goal
	 * @param nodeSearched
	 * @param Comparator (AStarnodeComparator or NodeComparator)
	 * @return path from start to goal
	 */
	
	public List<GeographicPoint> GenericSearch(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, Comparator<MapNode> comp, boolean useTravelTime, boolean rushHour)
	{
		// Setup - check validity of inputs
		int numVisited = 0 ;
		double pathDistance;
		double travelDistance;
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}		
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(comp);
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		// Set distance for infinity for all nodes
		for (GeographicPoint pt : pointNodeMap.keySet())
		{
			//System.out.println("Setting infinite-distance for "  + pt);
			MapNode n = pointNodeMap.get(pt);
			n.setDistance(Double.POSITIVE_INFINITY);
		}
		// Add startnode with distance 0
		startNode.setDistance(0);
		toExplore.offer(startNode);	
		MapNode curr = null;


		while (!toExplore.isEmpty()) {
			// Remove first element from queue
			curr = toExplore.poll();
			numVisited ++;
			// hook for visualization
			nodeSearched.accept(curr.getLocation());

			// Check if curr is visited
			if (!visited.contains(curr)){
				// curr is not in visited set, so add it
				//System.out.println("Adding to queue : " + curr.getLocation());
				visited.add(curr);
				// Is this the endpoint ?
				if (curr.equals(endNode)) break;
				// Now process the neigbors
				Set<MapNode> neighbors = getNeighbors(curr);
				for (MapNode neighbor : neighbors) {
					// Check if distance through curr is shorter than current distance
					// pathDistance is either distance in km or distance in traveltime. 
					if (useTravelTime) { 
						pathDistance = curr.getDistance() + curr.getTravelTimeTo(neighbor, rushHour);
						travelDistance = curr.getActualDistance() + curr.getDistanceTo(neighbor);
					}
					else { 
						pathDistance = curr.getDistance() + curr.getDistanceTo(neighbor);
						travelDistance = pathDistance;
					}
					//System.out.print("Pathdistance = " + pathDistance);
					//System.out.println(" current is : " + neighbor.getDistance());
					if ( pathDistance < neighbor.getDistance()){
						//System.out.print("Found shorter route, updating " + neighbor.getLocation());
						//System.out.println(" and" + curr.getLocation());
						neighbor.setDistance(pathDistance);
						neighbor.setActualDistance(travelDistance);
						parentMap.put(neighbor, curr);
						toExplore.offer(neighbor);
					}
				}
			}
		}

		if (!curr.equals(endNode)) {
			System.out.println("No path found from " + start+ " to " + goal);
			return null;
		}

		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);
		//System.out.println("Explored nodes : " + visited.size());
		System.out.println("Visited : " + numVisited);
		System.out.println("Travel distance (km or time ): " + curr.getDistance());
		System.out.println("Actual distance (km) : " + curr.getActualDistance());
		//System.out.print("Path is :");
		//System.out.println(path);

		return path;
	}
	// Getter for useTravelTime
	public static boolean isUseTravelTime() {
		return useTravelTime;
	}
	// Setter for useTravelTime
	public void setUseTravelTime(boolean useTravelTime) {
		MapGraph.useTravelTime = useTravelTime;
	}
	// Getter for rushHour
	public static boolean isRushHour() {
		return rushHour;
	}
	// Setter for rushHour
	public void setRushHour(boolean rushHour) {
		MapGraph.rushHour = rushHour;
	}
	// main method for testing
	public static void main(String[] args)
	{
		/*  Basic testing
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		 */ 

		// more advanced testing
		/*
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");

		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		//theMap.printEdges();
		//theMap.printNodes();

		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());

		List<GeographicPoint> route = theMap.bfs(new GeographicPoint(1.0,1.0), 
				new GeographicPoint(8.0,-1.0));

		System.out.println(route);

		List<GeographicPoint> routeDijkstra = theMap.dijkstra(new GeographicPoint(1.0,1.0), 
				new GeographicPoint(8.0,-1.0));

		System.out.println(routeDijkstra);

		List<GeographicPoint> routeAStar = theMap.aStarSearch(new GeographicPoint(1.0,1.0), 
				new GeographicPoint(8.0,-1.0));

		System.out.println("AStar result : " + routeAStar);	

		System.out.println(" Equal = " + routeAStar.equals(routeDijkstra));
        */

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		//theMap.printEdges();
		//theMap.printNodes();

		//GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		//GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		useTravelTime = false;
		System.out.println("Route without traveltime" );
		List<GeographicPoint> routeDijkstra1 = theMap.dijkstra(start,end);
		System.out.println(routeDijkstra1);

		System.out.println("Route with traveltime (outside rushhour)" );
		rushHour = false;
		useTravelTime = true;
		List<GeographicPoint> routeDijkstra2 = theMap.dijkstra(start,end);
		System.out.println(routeDijkstra2);
		System.out.println("Route 1 equals route 2 " + routeDijkstra1.equals(routeDijkstra2));

		System.out.println("Route with traveltime (during rushhour)" );
		rushHour = true;
		List<GeographicPoint> routeDijkstra3 = theMap.dijkstra(start,end);
		System.out.println(routeDijkstra3);
		System.out.println("Route 1 equals route 3 " + routeDijkstra1.equals(routeDijkstra3));
		System.out.println("Route 2 equals route 3 " + routeDijkstra2.equals(routeDijkstra3));

		//List<GeographicPoint> routeAMap = theMap.aStarSearch(start,end);
		//System.out.println(routeAMap);

	}

}

