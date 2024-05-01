//
// CubicSpline.cc
// CubicSplineLib/
//
// Source file for the "CubicSpline" class. This object facilitates natural
// cubic spline interpolation. Once instantiated the
// constructor builds the spline polynomials on the intervals of the (x, y)
// data provided and retains them for later invocation. Parallelized using
// OpenMP.
//
// Copyright (C) Geoffrey Lentner 2015. All rights reserved.
// See LICENCE file. (GPL v2.0) 
//
// contact: Geoffrey Lentner, B.S.
//          Graduate Student / Researcher
//          102 Natural Science Building
//          Department of Physics & Astronomy
//          University of Louisville
//          Louisville, KY 40292 USA
//
// email:   geoffrey.lentner@louisville.edu
//
// updated: 2015-1-19 13:10:30 EST
//

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <exception>
// #include <omp.h>

#include <plan_manage/CubicSpline.h>

// primary constructor
template<class T>
CubicSpline<T>::CubicSpline( const std::vector<T>& _x_, const std::vector<T>& _y_,
    int parallel, bool sorted ){

    // check for valid input arguments
    if ( _x_.size( ) < 4 || _y_.size( ) < 4 )
        throw SplineError("[_x_.size( ) < 4 || _y_.size( ) < 4]");
    
    if ( _x_.size( ) != _y_.size( ) )
        throw SplineError("[_x_.size( ) != _y_.size( )]");
    
    if ( parallel < 1 )
        throw SplineError("[parallel < 1]");
    
    // if ( parallel > omp_get_max_threads( ) )
    //     throw SplineError("[parallel > omp_get_max_threads()]");
    
    // // set parallelism
    // omp_set_num_threads( parallel );
    
	// last element in array has index 'n'
	n = _x_.size( ) - 1;
   
	// local copy of the vectors
    x = _x_;
    y = _y_;
    
    // if not suppressed, ensure 'x' elements are in ascending order
    if ( !sorted ) quicksort( );
    
    // build the spline polynomials on the intervals
    build_splines( );
}

// copy constructor
template<class T>
CubicSpline<T>::CubicSpline( const CubicSpline<T>& aCubicSpline ){

    // copy over data
    n = aCubicSpline.n;
    x = aCubicSpline.x;
    y = aCubicSpline.y;
    a = aCubicSpline.a;
    b = aCubicSpline.b;
    k = aCubicSpline.k;
}

// recursive quicksort on member vectors 'x' and 'y'
template<class T>
void CubicSpline<T>::quicksort ( const long& left , const long& right ) {
    
	// local values
    long l = left, r = right;

	// assign the value of the pivot to the mid-point,
	// this is the best option for uniform distribution
    T pivot = x[ ( left + right ) / 2 ];
    
    // partition the arrays
    while ( l <= r ) {
        
        while ( x[l] < pivot ) l++;
        while ( x[r] > pivot ) r--;
        
        if ( l <= r ) {
            
            std::iter_swap( x.begin( ) + l, x.begin( ) + r );
            std::iter_swap( y.begin( ) + l, y.begin( ) + r );
            
            l++; r--;
        }
    }
    
    // recursively call quicksort while appropriate
    if ( left < r     ) quicksort ( left , r     );
    if (    l < right ) quicksort ( l    , right );
}

// meta quicksort with parallel threads on class member 'data'
template<class T>
void CubicSpline<T>::quicksort( ) {
    
    if ( false ) {
        
        // code needs revision
    
    } else quicksort( 0, n );
}

// build natural cubic spline polynomials
template<class T>
void CubicSpline<T>::build_splines( ){

	// coefficients for tridiagonal linear system
    a.resize( n + 1 );
    b.resize( n + 1 );
    c.resize( n + 1 );
    d.resize( n + 1 );
    
	// optimize for re-occurring quantities
	T diffx1, diffx2, diffy1, diffy2;

	diffx1 = x[1] - x[0];
	diffy1 = y[1] - y[0];

	// first values
	a[0] = 0.0;
	c[0] = 1.0 / diffx1;
	b[0] = 2.0 * c[0];
	d[0] = 3.0 * diffy1 / ( diffx1 * diffx1 );

	// solve for coefficients of linear system
	#pragma omp parallel for
	for ( long i = 1; i < n; i++ ){

		diffx1 = x[i  ] - x[i-1];
		diffx2 = x[i+1] - x[i  ];
		diffy1 = y[i  ] - y[i-1];
		diffy2 = y[i+1] - y[i  ];

		a[i] = 1.0 / diffx1;
		c[i] = 1.0 / diffx2;
		b[i] = 2.0 * (a[i] + c[i]);
		d[i] = 3.0 * ( diffy1 / ( diffx1 * diffx1 ) + diffy2 /
                      ( diffx2 * diffx2 ) );
	}

	// set last remaining values
	diffx2 = x[n] - x[n-1];
	diffy2 = y[n] - y[n-1];

	a[n] = 1.0 / diffx2;
	b[n] = 2.0 * a[n];
	c[n] = 0.0;
	d[n] = 3.0 * diffy2 / ( diffx2 * diffx2 );

	// transform coefficients for the "Thomas algorithm"
    c_prime.resize( n );
    d_prime.resize( n + 1 );
    
	c_prime[0] = c[0] / b[0];
	d_prime[0] = d[0] / b[0];

	for ( long i = 1; i < n; i++ ) {
	
		c_prime[i] = c[i] / ( b[i] - a[i] * c_prime[i-1] );
		d_prime[i] = (d[i] - a[i] * d_prime[i-1]) / (b[i] - a[i] * c_prime[i-1]); 
	}

	// last one
	d_prime[n] = (d[n] - a[n] * d_prime[n-1]) / (b[n] - a[n] * c_prime[n-1]);

	// solve the linear system for the derivative of the splines
    k.resize( n + 1 );

	// the last one first
	k[n] = d_prime[n];

	// the remaining are solved via back substitution
	for ( long i = n - 1; i >= 0; i-- )
		k[i] = d_prime[i] - c_prime[i] * k[i+1];

	// now define coefficients for splines given derivatives
	// and recycle old variable names
	#pragma omp parallel for
	for ( long i = 1; i <= n; i++ ){

		diffx1 = x[i] - x[i-1];
		diffy1 = y[i] - y[i-1];

		a[i] =  k[i-1] * diffx1 - diffy1;
		b[i] = -k[i]   * diffx1 + diffy1;; 
	}
}

template<class T>
std::vector<T> CubicSpline<T>::interpolate( const std::vector<T>& new_x,
    const bool& fixed, const T& value ) {

    std::vector<T> output; output.resize( new_x.size( ) );

	#pragma omp parallel for shared(new_x,output)
	for ( long i = 0; i < new_x.size( ); i++ )
        output[i] = interpolate( new_x[i], fixed, value );
    
    return output;
}

template<class T>
T CubicSpline<T>::interpolate( const T& new_x, const bool& fixed,
                              const T& value ) {
    
    T output;

    if ( new_x < x[0] || new_x > x[n] ){

        if ( fixed )
            output = value;

        else if ( new_x < x[0] )
            output = k[0] * ( new_x - x[0] ) + y[0];
        
        else
            output = k[n] * ( new_x - x[n] ) + y[n];
        
    } else {

        long j = 1; while( new_x > x[j] && j < n ) j++;

        T t = ( new_x - x[j-1] ) / ( x[j] - x[j-1] );

        output = (1 - t) * y[j-1] + t * y[j] + t * (1 - t) * ( a[j] *
                 (1 - t) + b[j] * t );
    }
    
    return output;
}

template class CubicSpline<float>;
template class CubicSpline<double>;
template class CubicSpline<long double>;

























