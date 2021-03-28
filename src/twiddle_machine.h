#ifndef TWIDDLE_MACHINE_H
#define TWIDDLE_MACHINE_H

#include <iostream>
#include <cmath>
#include <vector>

class TwiddleStateMachine {
public:
    enum State {
        STATE_1,     //Initializing. Fist evaluation of the error
        STATE_2,
        STATE_3, 
        STATE_4,
        STATE_5,
        STATE_6
    };

    /**
    * Constructor.
    */
    explicit TwiddleStateMachine( int numberOfParams, int n, std::vector< double > params, std::vector< double > initDp );

    /**
    * Destructor.
    */
    virtual ~TwiddleStateMachine();

    const std::vector< double > & params() const {
        return m_params;
    }

    bool update( double error, double action );

private:
    bool state1( double cte  );
    bool state2( double cte );
    bool state3( double cte );
    bool state4( double cte );
    bool state5( double cte );
    bool state6( double cte );

    void goToState1();
    void goToState2();
    void goToState3();
    void goToState4();
    void goToState5();
    void goToState6();

    State m_state;

    int m_numberOfParams;
    int m_iterations;
    int m_counter;

    double m_errSum;

    double m_bestErr;

    int m_paramBeinSweep;

    uint16_t m_iterationCounter;

    std::vector< double > m_params;
    std::vector< double > m_dp;

};

#endif  //TWIDDLE_MACHINE_H