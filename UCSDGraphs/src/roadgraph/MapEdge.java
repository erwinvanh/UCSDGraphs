/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author UCSD Intermediate Programming MOOC team
 *
 * A directed edge in a map graph from Node start to Node end
 */
class MapEdge 
{
	/** The name of the road */
	private String roadName;
	
	/** The type of the road */
	private String roadType;
	
	/** The two endpoints of the edge */
	private MapNode start;
	private MapNode end;
	
	
	/** The length of the road segment, in km */
	private double length;
	
	static final double DEFAULT_LENGTH = 0.01;
	// Project extension
	static final double DEFAULT_SPEED_FAST = 55.0;
	static final double DEFAULT_SPEED_SLOW = 30.0;
	static final double CITY_STREET_SPEED_FAST = 50.0;
	static final double CITY_STREET_SPEED_SLOW = 40.0;
	static final double RESIDENTIAL_SPEED_FAST = 40.0;
	static final double RESIDENTIAL_SPEED_SLOW = 30.0;
	static final double PRIMARY_SPEED_FAST = 65.0;
	static final double PRIMARY_SPEED_SLOW = 30.0;
	static final double SECONDARY_SPEED_FAST = 30.0;
	static final double SECONDARY_SPEED_SLOW = 28.0;
	static final double TERTIARY_SPEED_FAST = 30.0;
	static final double TERTIARY_SPEED_SLOW = 28.0;
	
	private double travelTimeFast;
	private double travelTimeSlow;
	
	/** Create a new MapEdge object
	 * 
	 * @param roadName
	 * @param n1  The point at one end of the segment
	 * @param n2  The point at the other end of the segment
	 * 
	 */
	MapEdge(String roadName, MapNode n1, MapNode n2) 
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	
	MapEdge(String roadName, String roadType, MapNode n1, MapNode n2) 
	{
		this(roadName, roadType, n1, n2, DEFAULT_LENGTH);
	}
	
	MapEdge(String roadName, String roadType,
			MapNode n1, MapNode n2, double length) 
	{
		this.roadName = roadName;
		start = n1;
		end = n2;
		this.roadType = roadType;
		this.length = length;
		switch (roadType){
		case "city street": 
			this.travelTimeFast = calcTravelTime ( length, CITY_STREET_SPEED_FAST);
			this.travelTimeSlow = calcTravelTime ( length, CITY_STREET_SPEED_SLOW);
			break;
		case "primary": 
			this.travelTimeFast = calcTravelTime ( length, PRIMARY_SPEED_FAST);
			this.travelTimeSlow = calcTravelTime ( length, PRIMARY_SPEED_SLOW);
			break;		
		case "residential": 
			this.travelTimeFast = calcTravelTime ( length, RESIDENTIAL_SPEED_FAST);
			this.travelTimeSlow = calcTravelTime ( length, RESIDENTIAL_SPEED_SLOW);
			break;			
		case "secondary": 
			this.travelTimeFast = calcTravelTime ( length, SECONDARY_SPEED_FAST);
			this.travelTimeSlow = calcTravelTime ( length, SECONDARY_SPEED_SLOW);
			break;			
		case "tertiary": 
			this.travelTimeFast = calcTravelTime ( length, TERTIARY_SPEED_FAST);
			this.travelTimeSlow = calcTravelTime ( length, TERTIARY_SPEED_SLOW);
			break;			
		default:
			this.travelTimeFast = calcTravelTime ( length, DEFAULT_SPEED_FAST);
			this.travelTimeSlow = calcTravelTime ( length, DEFAULT_SPEED_SLOW);
            break;
		}
		
	}
	/**
	 * Calculate travel time based upon length and speed
	 * @param length
	 * @param speed
	 * @return traveltime in minutes
	 */

	private double calcTravelTime(double length, double speed) {

		return (length / speed ) * 60;
	}

	// return the MapNode for the end point
	MapNode getEndNode() {
	   return end;
	}
	
	// return the location of the start point
	GeographicPoint getStartPoint()
	{
		return start.getLocation();
	}
	
	// return the location of the end point
	GeographicPoint getEndPoint()
	{
		return end.getLocation();
	}
	
	// return the length
	double getLength()
	{
		return length;
	}
	
	
	
	// return road name
	public String getRoadName()
	{
		return roadName;
	}

	// return road type
	public String getRoadType()
	{
		return roadType;
	}
	// given one node in an edge, return the other node
	MapNode getOtherNode(MapNode node)
	{
		if (node.equals(start)) 
			return end;
		else if (node.equals(end))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
	// return String containing details about the edge
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		toReturn += "\nTraveltime (fast) " + travelTimeFast;
		toReturn += "\nTraveltime (slow) " + travelTimeSlow;
		return toReturn;
	}
    // return the traveltime for this road based upon boolean for rushhour
	public double getTravelTime(boolean rushHour){
		if (rushHour) return this.travelTimeSlow ;
		else return this.travelTimeFast;
	}

}
