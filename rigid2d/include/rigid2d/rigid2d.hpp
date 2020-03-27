#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <math.h>
#include <iostream>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        return fabs(d1 - d2) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return PI/180*deg;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return 180/PI*rad;
    }

    /// \brief  turns any angle into the equivalent one between -Pi and Pi
    /// \param rad - input angle
    /// \return rad
    constexpr double normalize_angle(double rad){
        double rem=remainder(rad,2*PI);
        return rem;
    };

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    static_assert(almost_equal(normalize_angle(2*PI), 0), "normalize_angle failed");


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x, y;
        Vector2D (){x = 0.0; y = 0.0;}
        Vector2D (double x_in, double y_in){x = x_in;y = y_in;}
    };

    /// \brief A 3-Dimensional Twist Vector
    struct Twist2D
    {
        double w, vx, vy;
        Twist2D (){w = 0.0; vx = 0.0; vy = 0.0;}
        Twist2D (double w_in, double vx_in, double vy_in){w = w_in;vx = vx_in;vy = vy_in;}
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief output a 3 dimensional twist as [w, vx, vy]
    /// os - stream to output to
    /// t - the vector to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & t);

    /// \brief input a 3 dimensional twist
    /// is - stream from which to read
    /// t [out] - output vector
    std::istream & operator>>(std::istream & is, Twist2D & t);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief Create a transformation with a pose
        /// \param pose - the pose of the object(theta,x,y)
        Transform2D(Twist2D pose);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D using adjoint
        /// \param t - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D t) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief return the displacement
        /// \return the x, y, and theta values from your transform.
        Twist2D displacement();

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    private:
        float TransformMatrix[3][3]= {1,0,0,0,1,0,0,0,1};
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function can be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief add the right vector to the left vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the sum of the two vectors
    Vector2D operator+=(Vector2D & lhs, const Vector2D & rhs);

    /// \brief add two vectors together
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the sum of the two vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief cut the right vector to the left vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the difference of the two vectors
    Vector2D operator-=(Vector2D & lhs, const Vector2D & rhs);

    /// \brief cut the right vector to the left vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the difference of the two vectors
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply the right number to the left vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the product of the vector and the number
    Vector2D operator*=(Vector2D & lhs, const double rhs);

    /// \brief multiply the right number to the left vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the product of the vector and the number
    Vector2D operator*(Vector2D & lhs, const double rhs);

    /// \brief multiply the left number to the right vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the product of the two vectors
    Vector2D operator*=(const double lhs, Vector2D & rhs);

    /// \brief multiply the left number to the right vector
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the product of the two vectors
    Vector2D operator*(const double lhs, Vector2D & rhs);

    /// \brief normalize 2D Vector
    /// \param trans - input Vector to be normalized
    /// \return the normalized Vector
    Vector2D normalize(Vector2D vec);


    /// \brief compute the transformation corresponding to a rigid body following a constant twist for one time unit.
    /// \param twist - twist to be integrated
    /// \return the transformation
    Transform2D integrateTwist(Twist2D twist);

    /// \brief compute the length of a Vector2D
    /// \param vec - input vector
    /// \return length
    double length(Vector2D vec);

    /// \brief compute the distance of 2 Vector2D
    /// \param vec1,vec2 - input vector
    /// \return distance
    double distance(Vector2D vec1, Vector2D vec2);

    /// \brief compute the angle of a Vector2D (in radians)
    /// \param vec - input vector
    /// \return angle
    double angle(Vector2D vec);
}

#endif
