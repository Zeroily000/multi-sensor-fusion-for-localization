#pragma once
#ifndef BASE_HPP
#define BASE_HPP

#include <Eigen/Core>
namespace imu
{

/** @brief Simple container for a data item (e.g., timestamp + x, y, z accelerometers or
 *         gyroscopes reading */
template <typename T>
class Data
{
public:
    /** @brief Construct an uninitialized Data object */
    Data();

    /** @brief Destructor */
    ~Data();

    /** @brief Construct a Data object from a timestamp and three values */
    Data(T timestamp, T x, T y, T z);

    /** @brief Construct a Data object from a timestamp and an Eigen vector */
    Data(T timestamp, Eigen::Matrix<T, 3, 1> data);

    /** @brief Construct a Data object from a timestamp and an array with 3 elements */
    Data(T timestamp, T const * data);

    /** @brief Copy constructor */
    template <typename T2>
    explicit Data(Data<T2> const & other);

    /** @brief Copy assignment operator */
    template <typename T2>
    Data & operator=(Data<T2> const & other);

    /** @brief Move constructor */
    template <typename T2>
    explicit Data(Data<T2> && other) noexcept;

    /** @brief Move assignment operator */
    template <typename T2>
    Data & operator=(Data<T2> && other) noexcept;

    inline T const & timestamp() const;
    inline Eigen::Matrix<T, 3, 1> const & data() const;
    inline T const & operator() (Eigen::Index i) const;
    inline T const & x() const;
    inline T const & y() const;
    inline T const & z() const;
private:
    Eigen::Matrix<T, 3, 1> m_data;
    T m_timestamp;
};


}

#include "base_impl.hpp"
#endif //BASE_HPP
