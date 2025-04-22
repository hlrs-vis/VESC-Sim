#ifndef CYCLINGDYNAMICS_H
#define CYCLINGDYNAMICS_H

#include <cmath>  // For std::cos and std::sin

    struct BikeForces {
        double airResistance;
        double rollingResistance;
        double slopeForce;
        double totalForce;
    };

class CyclingDynamics {
public:
    // Bicycle and physical properties
    double mass = 75.0;      // Mass of rider + bike in kg
    double CdA = 0.5;        // Drag area in m²
    double Crr = 0.0033;     // Rolling resistance coefficient
    const double g = 9.81;   // Gravitational acceleration in m/s²
    double number_poles = 14.0;
    double gear_ratio = 3.57;
    double wheel_diameter = 0.622;  // 622 mm

    // Constructor
    CyclingDynamics(double mass = 75.0, double CdA = 0.5, double Crr = 0.0033, double number_poles = 14.0 , double gear_ratio = 3.6, double wheel_diameter = 0.622)
        : mass(mass), CdA(CdA), Crr(Crr), number_poles(number_poles), gear_ratio(gear_ratio), wheel_diameter(wheel_diameter) {}

    // Motor conversion from ERPM to speed and vice versa
    double speed(double erpm);
    double new_rpm(double speed);

    // Struct for bike forces


    // Method to calculate forces acting on the bike
    BikeForces calculateForces(double velocity, double slope);

private:
    // Private method to calculate the wheel circumference
    double getWheelCircumference() const;
};

#endif // CYCLINGDYNAMICS_H
