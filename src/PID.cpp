#include "PID.h"

PID::PID() :
    m_kp( 1.0 ),
    m_ki( 1.0 ),
    m_kd( 1.0 ),
    m_dt( 1.0 ),
    m_maxVal( 1.0 ),
    m_maxISum( 1.0 ),
    m_alpha( 1.0 ),
    m_PError( 0.0 ),
    m_IError( 0.0 ),
    m_DError( 0.0 ), 
    m_lastOutput( 0.0 ) {

}


PID::~PID() {

}


void PID::Init( double kp, double ki, double kd, double dt, double maxVal, double maxISum, double outputFilterAlpha ) {

    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    m_dt = dt;

    m_maxVal = maxVal;

    m_maxISum = maxISum;

    m_alpha = outputFilterAlpha;

}


void PID::UpdateConstants( double kp, double ki, double kd ) {

    if( m_ki != ki ) {
        m_IError = 0.0;
    }

    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

}


void PID::UpdateError( double cte ) {

    m_DError = (cte - m_PError)/m_dt;
    m_PError = cte;
    m_IError += cte*m_dt;

    //Adding an "anti windup" to prevent the i term to over sum
    m_IError = clamp( m_IError, m_maxISum );  
}


double PID::TotalError() {
    return m_PError + m_IError + m_DError;
}


double PID::GetOutput() {
    double pOutput = m_kp*m_PError;
    double iOutput = m_ki*m_IError;
    double dOutput = m_kd*m_DError;
    double output = -1.0*( pOutput + dOutput + iOutput );

    m_lastOutput = m_lastOutput*m_alpha + output*( 1.0 - m_alpha );

    return clamp( m_lastOutput );
}