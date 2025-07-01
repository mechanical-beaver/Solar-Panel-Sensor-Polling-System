/*
 * Author: @github.com/annadostoevskaya
 * Filename: avr_type_traits.h
 * Created: 01 Jul 2025 14:34:51
 * Last Update: 02 Jul 2025 02:22:36
 *
 * Description: Replace some type traits for AVR
 */

#ifndef _AVR_TYPE_TRAITS_H_
#define _AVR_TYPE_TRAITS_H_

// is_same<T, U>
template <typename T, typename U> struct is_same {
    static const bool value = false;
};

template <typename T> struct is_same<T, T> {
    static const bool value = true;
};

// enable_if<condition, T>
template <bool B, typename T = void> struct enable_if {};

template <typename T> struct enable_if<true, T> {
    typedef T type;
};

#endif // _AVR_TYPE_TRAITS_H_
