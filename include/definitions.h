#define I2C_SDA 19
#define I2C_SCL 23

#define LEDPin 22

#define Coord_x 50
#define Coord_y 37.5
#define Coord_z 50
#define Coord_e 15

#define debounceTollerance 50


//functions
#define mapf(x_in, in_min, in_max, out_min, out_max) ((x_in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
