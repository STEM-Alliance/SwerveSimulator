#include <iostream>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <wpi/array.h>
#include <frc/kinematics/ChassisSpeeds.h>

using std::cout;
using std::endl;
using namespace frc;
using namespace wpi;

double _xoffset;
double _yoffset;
double _module_states[8] = {0.0};

int calculate_cpp(double vx, double vy, double omega, double robot_angle)
{
    Translation2d w1(units::meter_t{_xoffset}, units::meter_t{_yoffset});
    Translation2d w2(units::meter_t{_xoffset}, units::meter_t{-_yoffset});
    Translation2d w3(units::meter_t{-_xoffset}, units::meter_t{_yoffset});
    Translation2d w4(units::meter_t{-_xoffset}, units::meter_t{-_yoffset});
    Translation2d center(units::meter_t{0}, units::meter_t{0});

    array<Translation2d, 4> wheels = {w1, w2, w3, w4};

    SwerveDriveKinematics sd(wheels);

    Rotation2d robotAngle = Rotation2d(units::radian_t(robot_angle));

    auto speeds = ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(vx),
                                                         units::meters_per_second_t(vy),
                                                         units::radians_per_second_t(omega),
                                                         robotAngle);


    auto states = sd.ToSwerveModuleStates(speeds, center);

    _module_states[0] = (double)states[0].speed;
    _module_states[1] = (double)states[0].angle.Degrees();
    _module_states[2] = (double)states[1].speed;
    _module_states[3] = (double)states[1].angle.Degrees();
    _module_states[4] = (double)states[2].speed;
    _module_states[5] = (double)states[2].angle.Degrees();
    _module_states[6] = (double)states[3].speed;
    _module_states[7] = (double)states[3].angle.Degrees();
    return 0;
}

extern "C" {

int init(double xoffset, double yoffset)
{
    _xoffset = xoffset;
    _yoffset = yoffset;
    return 0;
}

int calculate(double vx, double vy, double omega, double robot_angle)
{
    return calculate_cpp(vx, vy, omega, robot_angle);
}

double * get_states()
{
    return _module_states;
}

}
