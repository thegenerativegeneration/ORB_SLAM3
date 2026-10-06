/**
 * Declarations that let the ORB-SLAM3 / DBoW2 headers compile without Boost when
 * ORBSLAM3_NO_SERIALIZATION is defined. The serialize() member templates stay in the
 * headers but are never instantiated (atlas save/load is compiled out in System.cc),
 * so only the names they mention must exist.
 */
#ifndef ORBSLAM3_SERIALIZATION_STUB_H
#define ORBSLAM3_SERIALIZATION_STUB_H

#include <cstddef>

namespace boost {
namespace serialization {

class access;

template<class T>
struct array_wrapper
{
    T* data;
    std::size_t count;
};

template<class T>
array_wrapper<T> make_array(T* data, std::size_t count) { return array_wrapper<T>{data, count}; }

template<class Base, class Derived>
Base& base_object(Derived& d) { return static_cast<Base&>(d); }

} // namespace serialization
} // namespace boost

#endif
