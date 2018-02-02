import math;

def fmt(value):
    return "%.3f" % value

period = 4
radius = 1.5
timestep = 0.02
maxtime = period*1

with open('CircleNoFF.txt', 'w') as the_file:
    t=0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period) * radius;
        y = math.cos(t * 2 * math.pi / period) * radius;
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + "-1\n");
        t += timestep;
            
