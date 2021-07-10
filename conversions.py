from math import dist

def inches_to_meters(inches):
    return inches * 0.0254

def meters_to_inches(meters):
    return meters * 39.3701

def meters_to_feet(meters):
    return 3.28084 * meters

def liters_to_gallons(liters):
    return liters * 0.264172

def gallons_to_liters(gallons):
    return gallons * 3.785412534258

def knots_to_kmph(knots):
    return knots * 1.852

def radians_to_degrees(radians):
    return radians * 57.295779513082325225835265587528

def degrees_to_radians(degrees):
    return degrees * 0.01745329251994329576923690766743

def distance(p, q):
    # p and q are lists
    return math.dist(p,q)