#pragma once

/**
 * This defines a class that can visit another class of type T
 *
 * We generally use this to avoid the issue of cyclical dependencies between the visitor
 * class and what it's visiting.
 *
 * @tparam T The type of class that can be visited
 *
 * Example usage:
 * // Element.hpp
 * #pragma once
 * #include "visitor.h" // Change this path to the location of this file.
 * #include <iostream>
 *
 * class Element
 * {
 *    public:
 *     virtual void accept(Visitor<Element>& v)
 *     {
 *         v.visit(*this);
 *     }
 *     virtual void talk() {
 *         std::cout << "Element talking!\n";
 *     }
 *     virtual ~Element() = default;
 * };
 *
 * Taken from this stackoverflow answer:
 * https://stackoverflow.com/questions/55801803/how-to-write-includes-with-visitor-pattern-simple-example
 */
template<class T>
class Visitor {
   public:
    virtual void visit(T& item) = 0;
    virtual ~Visitor() = default;
};
