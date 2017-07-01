//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static double invSqrt(double x);
    double beta;				// algorithm gain
    double q0;
    double q1;
    double q2;
    double q3;	// quaternion of sensor frame relative to auxiliary frame
    double invSampleFreq;
    double roll;
    double pitch;
    double yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(double sampleFrequency) { invSampleFreq = 1.0 / sampleFrequency; }
    void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
    void updateIMU(double gx, double gy, double gz, double ax, double ay, double az);
    //double getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //double getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //double getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    double getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578;
    }
    double getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578;
    }
    double getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578 + 180.0;
    }
    double getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    double getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    double getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};
#endif

