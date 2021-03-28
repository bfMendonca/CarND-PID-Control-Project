#ifndef PID_H
#define PID_H

#include <iostream>
#include <cmath>

class PID {
public:
    /**
    * Constructor
    */
    explicit PID();

    /**
    * Destructor.
    */
    virtual ~PID();

    /**
    * Initialize PID.
    * @param (Kp_, Ki_, Kd_, dt_, max_val_, max_isum_, output_filter_alpha_ ) The initial PID coefficients
    */
    void Init( double kp, double ki, double kd, double dt, double maxVal, double maxISum, double outputFilterAlpha );


    /**
    * Update the PID constants
    * Change kp, ki and kd constants withtout reseting the controller
    */
    void UpdateConstants( double kp, double ki, double kd );

    /**
    * Update the PID error variables given cross track error.
    * @param cte The current cross track error
    */
    void UpdateError(double cte );

    /**
    * Calculate the total PID error.
    * @output The total PID error
    */
    double TotalError();


    /**
    * The the ootuput given the last error
    *
    */
    double GetOutput();

private:

    double clamp( double value, double limit ) {
        if( value > limit ) {
            return limit;
        }else if( value < -limit ) {
            return -limit;
        }

        return value;
    }

    double clamp( double value ) {
        return clamp( value, m_maxVal ) ;
    }

    /**
    * PID Coefficients
    */ 
    double m_kp;
    double m_ki;
    double m_kd;

    double m_dt;

    double m_maxVal;

    double m_maxISum;

    double m_alpha;

    /**
    * PID Errors
    */
    double m_PError;
    double m_IError;
    double m_DError;

    double m_lastOutput;

};

#endif  // PID_H