#include "CyclingDynamics.h"

// Convert ERPM to speed
double CyclingDynamics::speed(double erpm) {
    double wheel_circumference = getWheelCircumference();
    return erpm / number_poles / gear_ratio * wheel_circumference / 60;
}

// Convert speed to new RPM
double CyclingDynamics::new_rpm(double speed) {
    double wheel_circumference = getWheelCircumference();
    return speed * 60 / wheel_circumference * number_poles * gear_ratio;
}

// Method to calculate forces acting on the bike
BikeForces CyclingDynamics::calculateForces(double velocity, double slope) {
    BikeForces forces;

    // Air resistance: 0.5 * airDensity * CdA * v^2
    double airDensity = 1.225;  // kg/m³ (at 15°C at sea level)
    forces.airResistance = 0.5 * airDensity * CdA * velocity * velocity;

    // Rolling resistance: Crr * mass * g * cos(slope)
    forces.rollingResistance = Crr * mass * g * std::cos(std::atan(slope / 100.0));

    // Slope resistance: mass * g * sin(slope)
    forces.slopeForce = mass * g * std::sin(std::atan(slope / 100.0));

    // Total force: Sum of all resistances
    forces.totalForce = forces.airResistance + forces.rollingResistance + forces.slopeForce;

    return forces;
}

// Private method to calculate the wheel circumference
double CyclingDynamics::getWheelCircumference() const {
    return wheel_diameter * 3.14159;
}
