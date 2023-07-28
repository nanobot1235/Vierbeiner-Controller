#define I2C_SDA 19
#define I2C_SCL 23

#define LEDPin 22

#define Coord_x 50
#define Coord_y 37.5
#define Coord_z 50
#define Coord_q 15
#define Coord_w 15
#define Coord_e 15

#define Coord_f 75

#define debounceTollerance 250


//functions
#define mapf(x_in, in_min, in_max, out_min, out_max) ((x_in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
