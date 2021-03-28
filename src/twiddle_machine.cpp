#include "twiddle_machine.h"


TwiddleStateMachine::TwiddleStateMachine( int numberOfParams, int n, std::vector< double > params, std::vector< double > initDp ) :
	m_numberOfParams( numberOfParams ),
	m_iterations( n ),
	m_counter( 0 ),
	m_state( STATE_1 ),
	m_errSum( 0.0 ),
	m_paramBeinSweep( -1 ),
	m_iterationCounter( 0 ),
	m_params( params ),
	m_dp( initDp ) {
}


TwiddleStateMachine::~TwiddleStateMachine() {

}

bool TwiddleStateMachine::state1( double cte ) {
	m_errSum += cte;
	++m_counter;

	
	if( m_counter > m_iterations ) {

		m_bestErr = m_errSum/static_cast< double >( m_iterations );

		std::cout << "Best Error: Sum: " << m_bestErr << std::endl;

		//Just so that the goToState2 acts as expected
		m_paramBeinSweep = -1;
		goToState2();
	}
	return false;
}


bool TwiddleStateMachine::state2( double cte ) {
	m_errSum += cte;
	++m_counter;

	if( m_counter > m_iterations ) {
		m_errSum /= static_cast< double >( m_iterations );

		std::cout << "Best error: " << m_bestErr;
		std::cout << " Error error Sum: " << m_errSum << std::endl;

		if( m_errSum < m_bestErr ) {
			//So we got an improve, let's go back to state2 and sweep another parameter
			m_bestErr = m_errSum;
			goToState3();
		}else {
			//Ok, so it's worse, we need to sweep to oher side
			goToState4();
		}

	}
	return false;

}


bool TwiddleStateMachine::state3( double cte ) {
	
	//Starting all over again
	goToState2();
	return false;
}


bool TwiddleStateMachine::state4( double cte ) {
	m_errSum += cte;
	++m_counter;



	if( m_counter > m_iterations ) {
		m_errSum /= static_cast< double >( m_iterations );

		std::cout << "Best error: " << m_bestErr;
		std::cout << " Error error Sum: " << m_errSum << std::endl;

		if( m_errSum < m_bestErr ) {
			//So we got an improve, let's go back to state2 and sweep another parameter
			m_bestErr = m_errSum;
			goToState3();
		}else {
			//Ok, so it's worse, maybe a local minimum
			goToState5();
		}

	}
	return false;
}


bool TwiddleStateMachine::state5( double cte ) {
	goToState2();
	return false;
}


bool TwiddleStateMachine::state6( double cte ) {
	return false;
}



void TwiddleStateMachine::goToState1(){
}


void TwiddleStateMachine::goToState2(){
	std::cout << "Going to 2 " << std::endl;
	m_state = STATE_2;
	m_counter = 0;
	m_errSum = 0;
	
	//Changin which parameter will be swept
	m_paramBeinSweep = (m_paramBeinSweep + 1)%m_numberOfParams;

	//Changin parameter for iteration
	m_params[m_paramBeinSweep] += m_dp[m_paramBeinSweep];

	++m_iterationCounter;

	std::cout << "Iteration: " << m_iterationCounter << std::endl;

	std::cout << "Parameter: ";
	for( const auto & v: m_params ) {
		std::cout << v << " ";
	}

	std::cout << " || sweep delta: ";
	for( const auto & v: m_dp ) {
		std::cout << v << " ";
	}
	std::cout << std::endl;

}


void TwiddleStateMachine::goToState3(){
	std::cout << "Going to 3 " << std::endl;
	m_state = STATE_3;
	m_dp[m_paramBeinSweep] *= 1.1;
}


void TwiddleStateMachine::goToState4(){
	std::cout << "Going to 4 " << std::endl;
	m_state = STATE_4;
	m_counter = 0;
	m_errSum = 0;

	//We will sweep the same parameter, but for the other side
	m_params[m_paramBeinSweep] -= 2*m_dp[m_paramBeinSweep];

	std::cout << "Parameter: ";
	for( const auto & v: m_params ) {
		std::cout << v << " ";
	}

	std::cout << " || sweep delta: ";
	for( const auto & v: m_dp ) {
		std::cout << v << " ";
	}
	std::cout << std::endl;

}


void TwiddleStateMachine::goToState5(){
	std::cout << "Going to 5 " << std::endl;
	m_state = STATE_5;

	m_params[m_paramBeinSweep] += m_dp[m_paramBeinSweep];
	m_dp[m_paramBeinSweep] *= 0.9;

}


void TwiddleStateMachine::goToState6(){

}



bool TwiddleStateMachine::update( double error, double action ) {
	bool returnState = false;

	error = pow(error,2.0) + pow(action,2.0);

	switch( m_state ) {
		case STATE_1: {
			returnState = state1( error );
		}
		break;

		case STATE_2: {
			returnState = state2( error);
		}
		break;

		case STATE_3: {
			returnState = state3( error );
		}
		break;

		case STATE_4: {
			returnState = state4( error );
		}
		break;

		case STATE_5: {
			returnState = state5( error );
		}
		break;

		case STATE_6: {
			returnState = state6( error );
		}
		break;
	}

	return returnState;
}