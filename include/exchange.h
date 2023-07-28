

struct exchange{
	//for servo
		double MPU[2];
		double coords[2][3];
		double legCoords[4][3];

	//for sensor
		bool goal;
		unsigned long tps;
};