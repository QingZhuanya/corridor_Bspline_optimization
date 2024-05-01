//
// CubicSpline.hh
// CubicSplineLib/
//
// Header file for the "CubicSpline" class. This object facilitates natural
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

#ifndef _CubicSpline_H
#define _CubicSpline_H

#include <vector>
#include <string>

template<class T = double>
class CubicSpline {

public:

    CubicSpline( ) { }
    
    CubicSpline( const std::vector<T>& _x_, const std::vector<T>& _y_,
             int parallel = 1, bool sorted = false );
    
    CubicSpline( const CubicSpline& );

    ~CubicSpline( ) { }

    std::vector<T> interpolate( const std::vector<T>& new_x,
        const bool& fixed = false, const T& value = 0 );
    
    T interpolate( const T& new_x, const bool& fixed = false,
                  const T&value = 0 );
    
protected:
    
    void quicksort( const long& , const long& );
    
    void quicksort( );
    
    void build_splines( );

    std::vector<T> x, y;

    std::vector<T> a, b, c, d;

    std::vector<T> c_prime, d_prime;

    std::vector<T> k;

    int n;
    
private:
    
    class SplineError : public std::exception {
    
    public:
        
        explicit SplineError( const char* msg ):
            _msg( msg ) { }
        
        explicit SplineError( const std::string& msg ):
            _msg( msg ) { }
        
        virtual ~SplineError( ) throw( ) { }
        
        virtual const char *what( ) const throw( ) {
            
            return ( "SplineError: " + _msg ).c_str( );
        }
        
    protected:
        
        std::string _msg;
    };
};
    
#endif
































