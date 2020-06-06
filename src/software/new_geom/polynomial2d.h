#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/polynomial1d.h"

/**
 * A 2D polynomial, represented as two polynomials, x(t) and y(t)
 */
class Polynomial2d
{
   public:
    /**
     * Construct a 2D polynomial. This will construct the polynomial such that
     * it always returns the point (0,0).
     */
    Polynomial2d();

    /**
     * Construct a 2d polynomial from two underlying 1d polynomials
     *
     * Note that these polynomials do not necessarily need to be of the same order.
     *
     * @param poly_x x(t), defines the x-coordinate of this 2D polynomial
     * @param poly_y y(t), defines the y-coordinate of this 2D polynomial
     */
    Polynomial2d(Polynomial1d poly_x, Polynomial1d poly_y);

    /**
     * Construct a 2d polynomial that passes through the given points
     *
     * The created polynomial will be at the first point at t=0, and at the last point at
     * t=1, with the other points linearly interpolated in between; ie. if we had three
     * points, the second point would be at t=0.5
     *
     * @param points A series of points the created polynomial will pass through. There
     *               must be at least two points.
     * @throw std::invalid_argument If there are less then two points
     */
    Polynomial2d(std::vector<Point> points);

    // TODO: should this be `getValueAt`? Update here and anywhere else...
    /**
     * Calculates the value of polynomial evaluated at value val
     *
     * @param val value to evaluate polynomial
     *
     * @return value of polynomial evaluated at value val
     */
    Point valueAt(double val) const;

    /**
     * Get the polynomial x(t) underlying this 2d Polynomial
     * @return The polynomial x(t) underlying this 2d Polynomial
     */
    Polynomial1d getPolyX() const;

    /**
     * Get the polynomial y(t) underlying this 2d Polynomial
     * @return The polynomial y(t) underlying this 2d Polynomial
     */
    Polynomial1d getPolyY() const;

   private:
    Polynomial1d poly_x;
    Polynomial1d poly_y;
};
