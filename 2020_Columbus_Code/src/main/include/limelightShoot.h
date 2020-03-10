#ifndef LIMELIGHT_SHOOT
#define LIMELIGHT_SHOOT

#include "commonVariables.h"
#include "Robot.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

double limelightShoot(double limelightSpeed, bool m_LimelightHasTarget)
{
    shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double ty = table->GetNumber("ty", 0.0);
    double tv = table->GetNumber("tv", 0.0);

    if (btnA0)
    {
        if (tv < 1.0)
        {
            m_LimelightHasTarget = false;
        }
        else
        {
            cout << "target " << endl;
            m_LimelightHasTarget = true;
        }
        if (m_LimelightHasTarget)
        {
            cout << "ty = " << ty << endl;
            if (ty > 0)
            {
                limelightSpeed = 0;
            }
            else if (ty < 0 && ty >= -0.49)
            {
                limelightSpeed = 6300;
            }
            else if (ty < -0.49 && ty >= -6.17)
            {
                cout << "in" << endl;
                limelightSpeed = 5500;
            }
            else if (ty < -6.17 && ty >= -7.87)
            {
                limelightSpeed = 5400;
            }
            else if (ty < -7.17 && ty >= -8.86)
            {
                limelightSpeed = 5500;
            }
            else if (ty < -8.86 && ty >= -13.55)
            {
                limelightSpeed = 5600;
            }
            else if (ty < -13.55 && ty >= -14.28)
            {
                limelightSpeed = 5900;
            }
            else if (ty < -14.28 && ty >= 15)
            {
                limelightSpeed = 6300;
            }
            else
            {
                limelightSpeed = 0;
            }
            return limelightSpeed;
            cout << "limelightShootSpeed = " << limelightSpeed << endl;
        }
    }
}

#endif