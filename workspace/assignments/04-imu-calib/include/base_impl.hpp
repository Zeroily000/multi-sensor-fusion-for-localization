#pragma once
#ifndef BASE_IMPL_HPP
#define BASE_IMPL_HPP

namespace imu {

template <typename T>
Data<T>::Data() = default;


template <typename T>
Data<T>::~Data() = default;


template <typename T>
Data<T>::Data(T timestamp, T x, T y, T z) : m_timestamp(timestamp), m_data(x, y, z) {}


template <typename T>
Data<T>::Data(T timestamp, Eigen::Matrix<T, 3, 1> data) : m_timestamp(timestamp), m_data(std::move(data)) {}


template <typename T>
Data<T>::Data(T timestamp, T const *const data) : m_timestamp(timestamp), m_data(data[0], data[1], data[2]) {}


template <typename T>
template <typename T2>
Data<T>::Data(Data <T2> const & other) :
    m_timestamp(static_cast<T>(other.m_timestamp)),
    m_data(other.m_data.template cast<T>())
{}


template <typename T>
template <typename T2>
Data <T> & Data<T>::operator=(Data <T2> const & other) {
    m_timestamp = static_cast<T>(other.m_timestamp);
    m_data = other.m_data.template cast<T>();
}

template <typename T>
template <typename T2>
Data<T>::Data(Data <T2> && other) noexcept :
    m_timestamp(static_cast<T>(other.m_timestamp)),
    m_data(std::move(other.m_data.template cast<T>()))
{}


template <typename T>
template <typename T2>
Data <T> & Data<T>::operator=(Data <T2> && other) noexcept {
    m_timestamp = static_cast<T>(other.m_timestamp);
    m_data = std::move(other.m_data.template cast<T>());
}


template <typename T>
T const & Data<T>::timestamp() const
{
    return m_timestamp;
}


template <typename T>
inline Eigen::Matrix<T, 3, 1> const & Data<T>::data() const
{
    return m_data;
}


template <typename T>
inline T const & Data<T>::operator() (Eigen::Index i) const
{
    return m_data[i];
}

template <typename T>
inline T const & Data<T>::x() const
{
    return m_data[0];
}

template <typename T>
inline T const & Data<T>::y() const
{
    return m_data[1];
}

template <typename T>
inline T const & Data<T>::z() const
{
    return m_data[2];
};


}
#endif //BASE_IMPL_HPP
